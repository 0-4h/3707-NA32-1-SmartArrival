#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ca.h"

// Support older secrets.h files that omit the optional authentication token.
#ifndef NTFY_TOKEN
#define NTFY_TOKEN ""
#endif

// Send HTTPS requests in a separate task to avoid blocking local control or MQTT.
namespace alerts {
struct Message { char body[180]; };
static QueueHandle_t queue;
static void task(void*) {
  Message message{};
  for (;;) {
    if (xQueueReceive(queue, &message, portMAX_DELAY) != pdTRUE) continue;
    // Retry this message every 60 seconds until HTTP succeeds. Later messages wait; a full queue rejects new messages.
    bool sent = false;
    while (!sent) {
      if (WiFi.status() == WL_CONNECTED && time(nullptr) >= 1700000000) {
        WiFiClientSecure tls;
        tls.setCACert(ROOT_CA); tls.setHandshakeTimeout(5);
        HTTPClient http;
        http.setConnectTimeout(5000); http.setTimeout(5000);
        if (http.begin(tls, NTFY_URL)) {
          http.addHeader("Content-Type", "text/plain; charset=utf-8");
          http.addHeader("Title", "SmartArrival sensor error");
          http.addHeader("Tags", "warning");
          if (strlen(NTFY_TOKEN)) http.addHeader("Authorization", String("Bearer ") + NTFY_TOKEN);
          int code = http.POST(String(message.body));
          sent = code >= 200 && code < 300;
          Serial.printf("NTFY_HTTP=%d\n", code);
          http.end();
        }
      }
      if (!sent) vTaskDelay(pdMS_TO_TICKS(60000));
    }
  }
}
static void begin() {
  if (strncmp(NTFY_URL, "https://", 8) != 0) {
    Serial.println("NTFY_NOT_CONFIGURED"); return;
  }
  queue = xQueueCreate(8, sizeof(Message));
  if (!queue) { Serial.println("NTFY_QUEUE_ERROR"); return; }
  if (xTaskCreate(task, "ntfy", 8192, nullptr, 1, nullptr) != pdPASS) {
    vQueueDelete(queue); queue = nullptr; Serial.println("NTFY_TASK_ERROR");
  }
}
// Callers must detect errors. This helper does not monitor sensors by itself.
static void send(const char* node, const char* error) {
  Message message{};
  snprintf(message.body, sizeof(message.body), "%s: %s. Sensor error. Automatic control is paused. Check the sensor. UTC=%lu",
           node, error, (unsigned long)time(nullptr));
  Serial.println(message.body);
  if (!queue) { Serial.println("NTFY_UNAVAILABLE"); return; }
  if (xQueueSend(queue, &message, 0) != pdTRUE) Serial.println("NTFY_QUEUE_FULL");
}
}
