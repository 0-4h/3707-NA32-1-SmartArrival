#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <time.h>
#include <Preferences.h>
#include <esp_system.h>
#include <initializer_list>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config.h"
#include "ca.h"
#include "control.h"
#include "mode_button.h"
#include "notify.h"

// Pass commands and sensor snapshots between local and network tasks.
sa::Controller controller;
DHT dht(DHT_PIN, DHT22);
sa::ModeButton modeButton;
struct Request { sa::Command command; uint32_t value; bool work; };
Preferences eventStore, modeStore;
bool modeStoreReady = false;
uint32_t workNotBefore = 0;
uint32_t lastWorkEpoch = 0;
bool eventStoreReady = false;
struct Snapshot {
  float temperature, humidity, distance;
  bool tempOK, distanceOK, light, ac;
  char status[96];
};
QueueHandle_t commands, snapshots;

// Validate MQTT commands and forward valid commands to the local task.
void mqttCallback(char* topic, byte* bytes, unsigned int length) {
  if (String(topic) == String(AIO_USERNAME) + "/feeds/sa-work-idle") {
    uint32_t epoch = 0; bool testEvent = false;
    time_t utc = time(nullptr);
    if (!eventStoreReady || !sa::parseEvent((const char*)bytes, length, epoch, &testEvent) ||
        !sa::fresh(epoch, (uint32_t)utc, EVENT_MAX_AGE_SECONDS) || epoch <= lastWorkEpoch) return;
    // Test events bypass only the weekday and work-time check here. Age and replay checks still apply.
    time_t eventTime = epoch; struct tm local{};
    if (!localtime_r(&eventTime, &local) ||
        (!testEvent && !sa::workWindow(local.tm_wday, local.tm_hour * 60 + local.tm_min, WORK_END_MINUTE))) return;
    if (!uxQueueSpacesAvailable(commands)) { Serial.println("CMD_QUEUE_FULL"); return; }
    // Record the event before execution to prevent replay after reboot.
    if (eventStore.putUInt("last", epoch) != sizeof(uint32_t)) {
      Serial.println("EVENT_STORE_ERROR"); return;
    }
    // The timestamp is consumed before the controller accepts the task. A later mode or task rejection does not make this event usable again.
    lastWorkEpoch = epoch;
    Request request{}; request.work = true; request.value = epoch;
    if (xQueueSend(commands, &request, 0) != pdTRUE) Serial.println("CMD_QUEUE_FULL");
    return;
  }
  if (String(topic) != String(AIO_USERNAME) + "/feeds/sa-command") {
    Serial.println("BROKER_NOTICE: check the IO monitor"); return;
  }
  if (length == 0 || length >= 32) return;
  char text[32]; memcpy(text, bytes, length); text[length] = 0;

  // Reject messages containing null bytes within the payload length.
  if (strlen(text) != length) return;
  Request r{};
  if (!strcmp(text, "MODE_LOCAL")) r.command = sa::Command::MODE_LOCAL;
  else if (!strcmp(text, "MODE_REMOTE")) r.command = sa::Command::MODE_REMOTE;
  else if (!strcmp(text, "DEPART")) r.command = sa::Command::DEPART;
  else if (!strcmp(text, "SYSTEM_OFF")) r.command = sa::Command::SYSTEM_OFF;
  else if (!strcmp(text, "RESET")) r.command = sa::Command::RESET;
  else if (!strcmp(text, "LIGHT_ON")) r.command = sa::Command::LIGHT_ON;
  else if (!strcmp(text, "LIGHT_OFF")) r.command = sa::Command::LIGHT_OFF;
  else if (!strcmp(text, "AC_ON")) r.command = sa::Command::AC_ON;
  else if (!strcmp(text, "AC_OFF")) r.command = sa::Command::AC_OFF;
  else if (!strcmp(text, "AUTO")) r.command = sa::Command::AUTO;
  else if (!strncmp(text, "ETA:", 4)) {
    if (!text[4]) return;
    uint32_t value = 0;
    for (size_t i = 4; i < length; ++i) {
      if (text[i] < '0' || text[i] > '9') return;
      value = value * 10 + (text[i] - '0');
      if (value > 7200) return;
    }
    if (value < 5) return;
    r.command = sa::Command::ETA; r.value = value;
  } else return;
  if (r.command == sa::Command::SYSTEM_OFF) {

    // Give SYSTEM_OFF priority over other queued commands.
    xQueueReset(commands);
    xQueueSendToFront(commands, &r, 0);
  } else if (xQueueSend(commands, &r, 0) != pdTRUE) {
    Serial.println("CMD_QUEUE_FULL");
  }
}

// Maintain Wi-Fi, TLS and MQTT connections and periodic cloud uploads.
void networkTask(void*) {
  WiFiClientSecure tls;
  tls.setCACert(ROOT_CA);
  tls.setHandshakeTimeout(5);
  PubSubClient mqtt(tls);
  mqtt.setServer("io.adafruit.com", 8883);
  mqtt.setCallback(mqttCallback);
  mqtt.setSocketTimeout(2);
  mqtt.setKeepAlive(30);
  mqtt.setBufferSize(384);
  String prefix = String(AIO_USERNAME) + "/feeds/";
  String cmdTopic = prefix + "sa-command";
  char clientId[48];
  snprintf(clientId, sizeof(clientId), "sa-%012llx-%08lx",
           (unsigned long long)ESP.getEfuseMac(), (unsigned long)esp_random());
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  configTzTime(LOCAL_TZ, "pool.ntp.org", "time.google.com");
  eventStoreReady = eventStore.begin("work-events", false);
  if (eventStoreReady) lastWorkEpoch = eventStore.getUInt("last", 0);
  else Serial.println("EVENT_STORE_ERROR");
  uint32_t wifiAt = millis(), retryAt = millis() - 10000, waitMs = 10000;
  uint32_t uploadAt = millis();
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (sa::elapsed(now, wifiAt) >= 15000) {
        wifiAt = now; WiFi.reconnect();
      }
    } else if (time(nullptr) < 1700000000) {

      // Wait for a valid clock before establishing TLS connections.
    } else if (!mqtt.connected()) {
      if (sa::elapsed(now, retryAt) >= waitMs) {
        retryAt = now;
        if (mqtt.connect(clientId, AIO_USERNAME, AIO_KEY)) {
          if (!mqtt.subscribe(cmdTopic.c_str(), 0) ||
              !mqtt.subscribe((prefix + "sa-work-idle").c_str(), 0)) mqtt.disconnect();
          else {
            mqtt.subscribe((String(AIO_USERNAME) + "/errors").c_str());
            mqtt.subscribe((String(AIO_USERNAME) + "/throttle").c_str());
            waitMs = 10000;
            Serial.println("MQTT_CONNECTED");

            // Do not request previous commands after reconnecting.
          }
        } else {
          Serial.printf("MQTT_ERROR %d\n", mqtt.state());
          waitMs = waitMs < 60000 ? waitMs * 2 : 60000;
          if (waitMs > 60000) waitMs = 60000;
        }
      }
    } else {
      mqtt.loop();
      if (sa::elapsed(now, uploadAt) >= 20000) {
        uploadAt = now;
        Snapshot s{};
        if (xQueuePeek(snapshots, &s, 0) == pdTRUE) {

          // Upload at most six data points every 20 seconds.
          auto publish = [&](const char* key, const char* value) {
            if (!mqtt.publish((prefix + key).c_str(), value, false))
              Serial.println("PUBLISH_FAILED");
          };
          char value[32];
          if (s.tempOK) {
            snprintf(value, sizeof(value), "%.1f", s.temperature); publish("sa-temperature", value);
            snprintf(value, sizeof(value), "%.1f", s.humidity); publish("sa-humidity", value);
          }
          if (s.distanceOK) {
            snprintf(value, sizeof(value), "%.1f", s.distance); publish("sa-distance", value);
          }
          publish("sa-light", s.light ? "1" : "0");
          publish("sa-ac", s.ac ? "1" : "0");
          publish("sa-status", s.status);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Share mode persistence and stale event filtering between cloud commands and the local button.
void applyCommand(sa::Command command, uint32_t now, uint32_t value = 0) {
  sa::Mode before = controller.mode;
  bool accepted = controller.command(command, now, value);
  if (accepted && before != controller.mode) {
    digitalWrite(MODE_LED_PIN, controller.mode == sa::Mode::LOCAL ? HIGH : LOW);
    // Reject events from the mode-change second or earlier.
    workNotBefore = (uint32_t)time(nullptr);
    if (!modeStoreReady || modeStore.putBool("local", controller.mode == sa::Mode::LOCAL) != 1)
      Serial.println("MODE_STORE_ERROR");
    Serial.printf("MODE_CHANGED: %s\n", controller.modeName());
  }
  Serial.println(accepted ? "CMD_OK" : "CMD_REJECTED");
}

// Initialize sensors, output pins, queues and the network task.
void setup() {
  Serial.begin(115200);
  modeStoreReady = modeStore.begin("control-mode", false);
  if (modeStoreReady) controller.restoreMode(modeStore.getBool("local", false));
  else Serial.println("MODE_STORE_ERROR");
  // Set output pins LOW at startup. The output code uses HIGH for a light-on request. The supplied diagram uses an npn relay; check the LED against L=0 and L=1.
  for (int pin : {LIGHT_RELAY_PIN, AC_PIN, TRIG_PIN}) {
    pinMode(pin, OUTPUT); digitalWrite(pin, LOW);
  }
  // Mode LED: on for local, off for remote; show the restored mode at startup.
  pinMode(MODE_LED_PIN, OUTPUT);
  digitalWrite(MODE_LED_PIN, controller.mode == sa::Mode::LOCAL ? HIGH : LOW);
  pinMode(ECHO_PIN, INPUT); pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  dht.begin(); controller.begin(millis());
  alerts::begin();
  commands = xQueueCreate(8, sizeof(Request));
  snapshots = xQueueCreate(1, sizeof(Snapshot));
  if (!commands || !snapshots) { Serial.println("QUEUE_ERROR"); abort(); }

  // Run network waits on core 0 to avoid blocking the local control loop.
  if (xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0) != pdPASS) {
    Serial.println("TASK_ERROR"); abort();
  }
  Serial.println("READY: d=depart x=stop r=reset e=ETA5 a=auto");
}

// Read sensors, evaluate local rules and drive outputs.
void loop() {
  static uint32_t tempAt = 0, rangeAt = 0, snapshotAt = 0;
  static sa::State previous = sa::State::IDLE;
  static bool injectFault = false;
  static const char* previousError = nullptr;
  uint32_t now = millis();
  if (sa::elapsed(now, tempAt) >= 2200) {
    tempAt = now;
    float h = dht.readHumidity(), t = dht.readTemperature();
    controller.tempSample(injectFault ? NAN : t, h, millis());
  }
  if (sa::elapsed(now, rangeAt) >= controller.distancePeriod(now)) {
    rangeAt = now;
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); digitalWrite(TRIG_PIN, LOW);

    // Wait at most 25 milliseconds for an ultrasonic echo.
    unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000);
    controller.distanceSample(us ? us / 58.0f : NAN, millis());
  }
  now = millis();
  Request r{};
  while (xQueueReceive(commands, &r, 0) == pdTRUE) {
    if (r.work) {
      time_t utc = time(nullptr), eventTime = r.value; struct tm local{};
      bool accepted = r.value > workNotBefore && sa::fresh(r.value, (uint32_t)utc, EVENT_MAX_AGE_SECONDS) &&
        localtime_r(&eventTime, &local) &&
        controller.startWork(now, ((uint32_t)utc - r.value) * 1000, SESSION_SECONDS * 1000);
      Serial.println(accepted ? "WORK_SCHEDULE_OK" : "WORK_SCHEDULE_REJECTED");
    } else {
      applyCommand(r.command, now, r.value);
    }
  }
  // Serial commands use one character. e changes ETA to five seconds; f and n work only when DEMO_FAULT_COMMANDS is true.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd') controller.command(sa::Command::DEPART, now);
    if (c == 'x') controller.command(sa::Command::SYSTEM_OFF, now);
    if (c == 'r') controller.command(sa::Command::RESET, now);
    if (c == 'e') controller.command(sa::Command::ETA, now, 5);
    if (c == 'a') controller.command(sa::Command::AUTO, now);
    if (DEMO_FAULT_COMMANDS && c == 'f') injectFault = true;
    if (DEMO_FAULT_COMMANDS && c == 'n') injectFault = false;
  }

  // Toggle mode on each stable press; release and holding do not retrigger.
  if (modeButton.sample(digitalRead(MODE_BUTTON_PIN) == LOW, now)) {
    applyCommand(controller.mode == sa::Mode::LOCAL ? sa::Command::MODE_REMOTE
                                                  : sa::Command::MODE_LOCAL, now);
  }
  // Queue an alert only for a new or changed error. Recovery is printed to serial; it is not sent to ntfy.
  const char* error = controller.sensorError(now);
  if (error && (!previousError || strcmp(error, previousError))) alerts::send("HOME", error);
  if (!error && previousError) Serial.println("HOME_SENSOR_RECOVERED");
  previousError = error;
  controller.tick(now);
  digitalWrite(LIGHT_RELAY_PIN, controller.light ? HIGH : LOW);
  digitalWrite(AC_PIN, controller.ac);
  if (previous != controller.state) {
    Serial.printf("%lu,%s,%s\n", (unsigned long)now, sa::name(controller.state), controller.reason);
    previous = controller.state;
  }
  if (sa::elapsed(now, snapshotAt) >= 1000) {
    snapshotAt = now;
    Snapshot s{};
    s.temperature = controller.temperature; s.humidity = controller.humidity; s.distance = controller.distance;
    // If either sensor is unhealthy, suppress all sensor feeds. Output and status feeds are still sent. Output fields are commands, not measured feedback.
    s.tempOK = controller.tempOK && controller.healthy(now);
    s.distanceOK = controller.distanceOK && controller.healthy(now);
    s.light = controller.light; s.ac = controller.ac;
    snprintf(s.status, sizeof(s.status), "%s | %s | %s | %lus | present=%d | %s",
             controller.modeName(), sa::name(controller.state), controller.reason, (unsigned long)(now / 1000), controller.present, error ? error : "OK");
    xQueueOverwrite(snapshots, &s);
    Serial.printf("%s T=%.1f RH=%.1f D=%.1f L=%d AC=%d\n",
                  s.status, s.temperature, s.humidity, s.distance, s.light, s.ac);
  }
  vTaskDelay(pdMS_TO_TICKS(5));
}