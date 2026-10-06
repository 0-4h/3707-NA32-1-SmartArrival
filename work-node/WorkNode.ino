#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include <Preferences.h>
#include <esp_system.h>
#include "config.h"
#include "schedule.h"
#include "notify.h"
#include "mode_button.h"

sa::DeskIdleDetector detector;
Preferences dayStore;
bool dayStoreReady = false;
struct Departure { uint32_t epoch; bool test; };
QueueHandle_t departures;
sa::ModeButton testButton;

// Send departure events over MQTT/TLS. This node does not send raw PIR readings or receive home commands.
void networkTask(void*) {
  WiFiClientSecure tls; tls.setCACert(ROOT_CA); tls.setHandshakeTimeout(5);
  PubSubClient mqtt(tls); mqtt.setServer("io.adafruit.com", 8883);
  mqtt.setSocketTimeout(2); mqtt.setKeepAlive(30);
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  configTzTime(LOCAL_TZ, "pool.ntp.org", "time.google.com");
  String topic = String(AIO_USERNAME) + "/feeds/sa-work-idle";
  char id[48];
  snprintf(id, sizeof(id), "work-%012llx-%08lx", (unsigned long long)ESP.getEfuseMac(), (unsigned long)esp_random());
  uint32_t retryAt = millis() - 10000, wifiAt = millis(), waitMs = 10000;
  uint32_t pending = 0, sentAt = 0;
  bool pendingTest = false;
  for (;;) {
    uint32_t now = millis();
    Departure event{};
    // One queued event is allowed. A newly received event replaces the event being resent.
    if (xQueueReceive(departures, &event, 0) == pdTRUE) {
      pending = event.epoch; pendingTest = event.test; sentAt = now - 15000;
    }
    uint32_t utc = (uint32_t)time(nullptr);
    if (pending && !sa::fresh(pending, utc, EVENT_MAX_AGE_SECONDS)) {
      Serial.println("EVENT_RETRY_WINDOW_END"); pending = 0;
    }
    if (WiFi.status() != WL_CONNECTED) {
      if (now - wifiAt >= 15000) { wifiAt = now; WiFi.reconnect(); }
    } else if (utc >= 1700000000) {
      if (!mqtt.connected() && now - retryAt >= waitMs) {
        retryAt = now;
        if (mqtt.connect(id, AIO_USERNAME, AIO_KEY)) { waitMs = 10000; Serial.println("WORK_MQTT_CONNECTED"); }
        else waitMs = waitMs < 30000 ? waitMs * 2 : 60000;
      }
      if (mqtt.connected()) {
        mqtt.loop();
        // Resend the same event; the home node rejects duplicate timestamps.
        if (pending && now - sentAt >= 15000) {
          sentAt = now;
          char text[48]; snprintf(text, sizeof(text), "v2|%s|%lu", pendingTest ? "work-test" : "work-desk", (unsigned long)pending);
          // EVENT_SENT reports publish success in this client. It is not an acknowledgement that the home node ran the task.
          Serial.println(mqtt.publish(topic.c_str(), text, false) ? "EVENT_SENT" : "EVENT_SEND_FAILED");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
void setup() {
  Serial.begin(115200);
  dayStoreReady = dayStore.begin("desk-idle", false);
  if (dayStoreReady) detector.firedDay = dayStore.getUInt("day", 0);
  else Serial.println("DAY_STORE_ERROR");
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  pinMode(TEST_LEAVE_PIN, INPUT_PULLUP);
  pinMode(TEST_MOTION_PIN, INPUT_PULLUP);
  departures = xQueueCreate(1, sizeof(Departure));
  if (!departures) abort();
  // The alert task is available, but this firmware has no alerts::send call and cannot detect a disconnected PIR from LOW alone.
  alerts::begin();
  if (xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0) != pdPASS) abort();
  Serial.println("WORK_READY");
}
void loop() {
  static uint32_t sampleAt = 0;
  uint32_t now = millis();
  bool forcedMotion = digitalRead(TEST_MOTION_PIN) == LOW;
  // Test leave skips weekday, work time, idle time and the daily limit. It requires a valid clock and a released continuous-motion switch; real PIR HIGH does not block it.
  if (testButton.sample(digitalRead(TEST_LEAVE_PIN) == LOW, now)) {
    time_t utc = time(nullptr);
    static uint32_t lastTest = 0;
    if (forcedMotion) Serial.println("TEST_BLOCKED_MOTION");
    else if (utc < 1700000000) Serial.println("TEST_CLOCK_NOT_READY");
    else if ((uint32_t)utc <= lastTest) Serial.println("TEST_WAIT_NEXT_SECOND");
    else {
      // Use a new timestamp for each accepted test press. Tests do not change the normal detector or its saved date.
      Departure event{(uint32_t)utc, true};
      if (xQueueSend(departures, &event, 0) == pdTRUE) {
        lastTest = event.epoch;
        Serial.println("TEST_LEAVE_QUEUED");
      } else Serial.println("WORK_QUEUE_FULL");
    }
  }
  if (now - sampleAt >= WORK_SAMPLE_MS) {
    sampleAt = now;
    time_t epoch = time(nullptr); struct tm local{};
    bool validClock = epoch >= 1700000000 && localtime_r(&epoch, &local);
    bool workingDay = validClock && sa::weekday(local.tm_wday);
    // Ignore PIR input on weekends. A LOW signal alone cannot confirm sensor health.
    bool motion = workingDay && (forcedMotion || digitalRead(PIR_PIN) == HIGH);
    // Normal detection starts on weekdays at 18:00. Earlier inactivity is not counted. A restart starts a new idle interval.
    bool enabled = workingDay && local.tm_hour * 60 + local.tm_min >= WORK_END_MINUTE;
    uint32_t day = validClock ? (local.tm_year + 1900) * 10000 + (local.tm_mon + 1) * 100 + local.tm_mday : 0;
    if (detector.sample(motion, enabled, day, now, IDLE_SECONDS * 1000)) {
      // The detector has already used its daily trigger. A storage or queue failure does not retry that normal trigger during this run.
      // Save the date before sending to prevent another trigger after reboot; skip sending if saving fails.
      if (!dayStoreReady || dayStore.putUInt("day", day) != sizeof(uint32_t)) {
        Serial.println("DAY_STORE_ERROR");
      } else {
        Departure value{(uint32_t)epoch, false};
        if (xQueueSend(departures, &value, 0) != pdTRUE) Serial.println("WORK_QUEUE_FULL");
        else Serial.println("WORK_IDLE_QUEUED");
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}