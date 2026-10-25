/*
 * Multi-Tasking RTOS Firmware Scheduler  (ESP32 + FreeRTOS)
 *
 *  SensorTask  (prio 3, 100 ms periodic) -> reads sensor, publishes data
 *  DisplayTask (prio 2, 500 ms periodic) -> shows data on SSD1306 OLED
 *  AlarmTask   (prio 4, event driven)    -> activates buzzer/LED on critical condition
 *
 *  IPC / synchronization:
 *   - displayQueue : length-1 "mailbox" (xQueueOverwrite) -> latest reading for display
 *   - alarmQueue   : event queue (readings + ISR button events) -> AlarmTask blocks on it
 *   - stateMutex   : protects shared SystemState_t (priority inheritance)
 *   - serialMutex  : protects the Serial port (shared resource)
 *   - I2C/OLED is owned exclusively by DisplayTask (no sharing => no race)
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdarg.h>
// ---------------- Configuration (merged from config.h) ----------------
// ---------------- Pin mapping (ESP32 DevKit) ----------------
#define PIN_SENSOR      34   // Potentiometer (simulates temperature sensor, ADC1)
#define PIN_BUZZER      25   // Alarm buzzer
#define PIN_ALARM_LED   26   // Alarm LED
#define PIN_EMERG_BTN   27   // Manual emergency push-button (active LOW)
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define OLED_ADDR       0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

// ---------------- Sensor scaling ----------------
#define ADC_MAX             4095.0f
#define TEMP_FULL_SCALE_C   100.0f   // pot at max = 100 C

// ---------------- Alarm thresholds (with hysteresis) ----------------
#define TEMP_ALARM_ON_C     70.0f    // alarm turns ON  at >= 70 C
#define TEMP_ALARM_OFF_C    65.0f    // alarm turns OFF at <= 65 C
#define ALARM_BLINK_MS      150

// ---------------- Periods (ms) ----------------
#define SENSOR_PERIOD_MS    100      // fast sampling => fast alarm detection
#define DISPLAY_PERIOD_MS   500

// ---------------- Priorities (higher number = higher priority) ----------------
#define PRIO_DISPLAY        2
#define PRIO_SENSOR         3
#define PRIO_ALARM          4        // highest: safety-critical

// ---------------- Stack sizes (bytes on ESP32) ----------------
#define STACK_SENSOR        3072
#define STACK_DISPLAY       4096
#define STACK_ALARM         3072

// All tasks pinned to ONE core so priority-based preemption is clearly visible
#define APP_CORE            1


// ------------------------- Types -------------------------
typedef struct {
  float       temperatureC;
  uint16_t    raw;
  TickType_t  tick;           // time the sample was taken
} SensorReading_t;

typedef enum { EVT_READING, EVT_BUTTON } AlarmEventType_t;

typedef struct {
  AlarmEventType_t type;
  float            temperatureC;
  TickType_t       tick;      // when the event was generated
} AlarmEvent_t;

typedef struct {
  SensorReading_t last;
  bool     alarmActive;
  bool     manualAlarm;
  uint32_t samples;
  uint32_t alarmCount;
  uint32_t lastLatencyMs;     // sensor sample -> alarm output latency
} SystemState_t;

// ------------------------- Globals -------------------------
static SystemState_t     gState = {};
static QueueHandle_t     displayQueue = NULL;
static QueueHandle_t     alarmQueue   = NULL;
static SemaphoreHandle_t stateMutex   = NULL;
static SemaphoreHandle_t serialMutex  = NULL;
static TaskHandle_t      hSensor = NULL, hDisplay = NULL, hAlarm = NULL;
static Adafruit_SSD1306  oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ------------------------- Helpers -------------------------
static inline uint32_t nowMs() { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

// Thread-safe logging (Serial is a shared resource)
static void logMsg(const char *fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.printf("[%7lu ms] %s\n", (unsigned long)nowMs(), buf);
    xSemaphoreGive(serialMutex);
  }
}

static inline void setAlarmOutputs(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
  digitalWrite(PIN_ALARM_LED, on ? HIGH : LOW);
}

// ------------------------- ISR: emergency button -------------------------
void IRAM_ATTR emergencyButtonISR() {
  static TickType_t lastTick = 0;
  TickType_t now = xTaskGetTickCountFromISR();
  if ((now - lastTick) < pdMS_TO_TICKS(200)) return;   // debounce
  lastTick = now;

  AlarmEvent_t ev = { EVT_BUTTON, 0.0f, now };
  BaseType_t hpTaskWoken = pdFALSE;
  xQueueSendFromISR(alarmQueue, &ev, &hpTaskWoken);
  if (hpTaskWoken) portYIELD_FROM_ISR();                // run AlarmTask immediately
}

// ------------------------- Task 1: Sensor acquisition -------------------------
static void sensorTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t n = 0;
  for (;;) {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += analogRead(PIN_SENSOR);   // simple averaging filter
    uint16_t raw = sum / 4;

    SensorReading_t r;
    r.raw          = raw;
    r.temperatureC = (raw / ADC_MAX) * TEMP_FULL_SCALE_C;
    r.tick         = xTaskGetTickCount();

    // 1) update shared state (short critical section)
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      gState.last = r;
      gState.samples++;
      xSemaphoreGive(stateMutex);
    }

    // 2) latest value for display (mailbox, never blocks)
    xQueueOverwrite(displayQueue, &r);

    // 3) event for alarm task (never blocks)
    AlarmEvent_t ev = { EVT_READING, r.temperatureC, r.tick };
    if (xQueueSend(alarmQueue, &ev, 0) != pdTRUE) logMsg("WARN: alarm queue full");

    if (++n % 10 == 0)
      logMsg("SENSOR  : raw=%4u  temp=%5.1f C", r.raw, r.temperatureC);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));   // fixed period, no drift
  }
}

// ------------------------- Task 2: Display updates -------------------------
static void displayTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t cycles = 0;
  for (;;) {
    SensorReading_t r = {};
    SystemState_t   s = {};
    xQueuePeek(displayQueue, &r, 0);
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      s = gState;
      xSemaphoreGive(stateMutex);
    }

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("RTOS Monitor");
    oled.setTextSize(2);
    oled.setCursor(0, 16);
    oled.printf("%5.1f C", r.temperatureC);
    oled.setTextSize(1);
    oled.setCursor(0, 40);
    oled.print(s.alarmActive ? "!! ALARM ACTIVE !!" : "Status: NORMAL");
    oled.setCursor(0, 54);
    oled.printf("Samples:%lu Alm:%lu", (unsigned long)s.samples, (unsigned long)s.alarmCount);
    oled.display();

    logMsg("DISPLAY : %5.1f C  alarm=%s", r.temperatureC, s.alarmActive ? "ON" : "off");

    if (++cycles % 10 == 0)
      logMsg("STACK   : min free bytes -> sensor=%u display=%u alarm=%u",
             (unsigned)uxTaskGetStackHighWaterMark(hSensor),
             (unsigned)uxTaskGetStackHighWaterMark(hDisplay),
             (unsigned)uxTaskGetStackHighWaterMark(hAlarm));

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
  }
}

// ------------------------- Task 3: Emergency alarm (highest priority) -------------------------
static void alarmTask(void *pv) {
  AlarmEvent_t ev;
  bool tempAlarm = false, manual = false, active = false, outLevel = false;

  for (;;) {
    // Idle: sleep forever until an event arrives (zero CPU use).
    // Alarm active: wake every ALARM_BLINK_MS to blink outputs.
    TickType_t wait = active ? pdMS_TO_TICKS(ALARM_BLINK_MS) : portMAX_DELAY;

    if (xQueueReceive(alarmQueue, &ev, wait) == pdTRUE) {
      if (ev.type == EVT_READING) {
        if (!tempAlarm && ev.temperatureC >= TEMP_ALARM_ON_C)       tempAlarm = true;
        else if (tempAlarm && ev.temperatureC <= TEMP_ALARM_OFF_C)  tempAlarm = false;  // hysteresis
      } else {
        manual = !manual;                                   // button toggles manual emergency
        logMsg("ALARM   : manual emergency button -> %s", manual ? "SET" : "CLEARED");
      }

      bool should = tempAlarm || manual;
      if (should != active) {
        active   = should;
        outLevel = active;
        setAlarmOutputs(outLevel);                          // hardware action FIRST

        uint32_t latency = (xTaskGetTickCount() - ev.tick) * portTICK_PERIOD_MS;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          gState.alarmActive   = active;
          gState.manualAlarm   = manual;
          gState.lastLatencyMs = latency;
          if (active) gState.alarmCount++;
          xSemaphoreGive(stateMutex);
        }
        if (active) logMsg("ALARM   : *** ACTIVATED *** temp=%.1f C  response latency=%lu ms",
                           ev.temperatureC, (unsigned long)latency);
        else        logMsg("ALARM   : cleared, back to NORMAL (temp=%.1f C)", ev.temperatureC);
      }
    } else if (active) {                                    // timeout while active => blink
      outLevel = !outLevel;
      setAlarmOutputs(outLevel);
    }
  }
}

// ------------------------- Setup / loop -------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_ALARM_LED, OUTPUT);
  pinMode(PIN_EMERG_BTN, INPUT_PULLUP);
  setAlarmOutputs(false);
  analogReadResolution(12);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed!");
  }
  oled.clearDisplay();
  oled.display();

  // RTOS objects
  stateMutex   = xSemaphoreCreateMutex();
  serialMutex  = xSemaphoreCreateMutex();
  displayQueue = xQueueCreate(1, sizeof(SensorReading_t));
  alarmQueue   = xQueueCreate(8, sizeof(AlarmEvent_t));
  if (!stateMutex || !serialMutex || !displayQueue || !alarmQueue) {
    Serial.println("RTOS object creation failed!");
    while (true) delay(1000);
  }

  logMsg("BOOT    : creating tasks (alarm=%d sensor=%d display=%d)",
         PRIO_ALARM, PRIO_SENSOR, PRIO_DISPLAY);

  xTaskCreatePinnedToCore(alarmTask,   "AlarmTask",   STACK_ALARM,   NULL, PRIO_ALARM,   &hAlarm,   APP_CORE);
  xTaskCreatePinnedToCore(sensorTask,  "SensorTask",  STACK_SENSOR,  NULL, PRIO_SENSOR,  &hSensor,  APP_CORE);
  xTaskCreatePinnedToCore(displayTask, "DisplayTask", STACK_DISPLAY, NULL, PRIO_DISPLAY, &hDisplay, APP_CORE);

  attachInterrupt(digitalPinToInterrupt(PIN_EMERG_BTN), emergencyButtonISR, FALLING);
  logMsg("BOOT    : scheduler running");
}

void loop() {
  vTaskDelete(NULL);   // Arduino loop task not needed; FreeRTOS tasks do all the work
}
