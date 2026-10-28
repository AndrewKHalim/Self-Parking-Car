// The runtime. Two FreeRTOS tasks, one core each. The 30 Hz control loop on core 1 touches I2C, servo and motor. The network side on core 0 waits on wifi and formats JSON, and neither may delay a sensor read. The supervisor belongs to the control task. The queue carries a copy of the telemetry, so the web side cannot read a half-updated state.

#include <Arduino.h>
#include <WiFi.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "dashboard_html.h"
#include "esp32_hal.h"
#include "supervisor.h"
#include "telemetry_ring.h"
#include "wifi_config.h"

// 30 Hz, which is what Config::dt says. The control law measures its own dt, but the stroke profiles assume a tick.
constexpr uint32_t CONTROL_PERIOD_MS = 33;

// Ten a second is enough to watch a maneuver that takes seconds, and keeps the socket work far below the tick rate.
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

// How long the web task waits for the configured network before falling back to its own access point.
constexpr uint32_t WIFI_TIMEOUT_MS = 15000;

// The core map. Control sits on core 1 with the Arduino loop task. The web task and async TCP sit on core 0 beside wifi and lwip, pinned there by sdkconfig and by CONFIG_ASYNC_TCP_RUNNING_CORE in platformio.ini.
constexpr BaseType_t CONTROL_CORE = 1;
constexpr BaseType_t WEB_CORE = 0;

// Stack sizes are bytes on the esp-idf port, not words. The control task holds the supervisor and reads three sensors per tick.
constexpr uint32_t CONTROL_STACK = 8192;
constexpr UBaseType_t CONTROL_PRIORITY = 5;
constexpr uint32_t WEB_STACK = 8192;
constexpr UBaseType_t WEB_PRIORITY = 2;

// The handoff. One item, overwritten by the producer, so the consumer cannot fall behind and a slow socket cannot delay the control loop.
static QueueHandle_t telemetryQueue = nullptr;

// The only word crossing the other way, and it is a request. The control task still refuses it before the calibration and during a run.
static std::atomic<bool> startRequest{false};

static AsyncWebServer server(80);
static AsyncWebSocket socket("/ws");
static Esp32Hal hal;

// The run, kept on the board so a reload or a dropped socket does not lose it. The control task writes one record per tick and the web side reads it behind the ring's lock.
static TelemetryRing runLog;

static const char* phaseName(Phase p) {
  switch (p) {
    case Phase::IDLE: return "idle";
    case Phase::BACK_IN: return "back_in";
    case Phase::CORRECT: return "correct";
    case Phase::DONE: return "done";
    case Phase::GIVEUP: return "giveup";
  }
  return "unknown";
}

static const char* strokeName(Stroke s) {
  switch (s) {
    case Stroke::NONE: return "none";
    case Stroke::ROTATE: return "rotate";
    case Stroke::TRANSLATE: return "translate";
  }
  return "unknown";
}

// One JSON object per message, numeric fields as integers. t is ms, x, rear, travel, clearance, gL and gR are mm, psi, cap, peak are milliradians, servo degrees. seq is the ring record this sample came from, so the page can line it up with the CSV.
static void publishTelemetry(const Telemetry& t, uint32_t seq) {
  if (socket.count() == 0) return;

  static char line[384];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"x\":%ld,\"psi\":%ld,\"rear\":%ld,\"servo\":%d,"
           "\"phase\":\"%s\",\"stroke\":\"%s\",\"strokes\":%d,"
           "\"cal\":%d,\"valid\":%d,\"travel\":%ld,\"cap\":%ld,"
           "\"clear\":%ld,\"peak\":%ld,\"gL\":%ld,\"gR\":%ld,\"seq\":%lu,"
           "\"rearFault\":%d,\"travelFault\":%d,\"outOfSpec\":%d}",
           (unsigned long)(t.t * 1000.0f),
           (long)lroundf(t.x * 1000.0f),
           (long)lroundf(t.psi * 1000.0f),
           (long)lroundf(t.rear * 1000.0f),
           (int)lroundf(t.servo),
           phaseName(t.phase), strokeName(t.stroke), t.strokes,
           (int)t.calibrated, (int)t.valid,
           (long)lroundf(t.travel * 1000.0f),
           (long)lroundf(t.psiCap * 1000.0f),
           (long)lroundf(t.clearance * 1000.0f),
           (long)lroundf(t.peakYaw * 1000.0f),
           (long)lroundf(t.gL * 1000.0f),
           (long)lroundf(t.gR * 1000.0f),
           (unsigned long)seq,
           (int)t.rearFault, (int)t.travelFault, (int)t.outOfSpec);
  socket.textAll(line);
}

// Slices the held run into fixed-width rows from the request offset, so the response is never buffered whole. first and total come from one snapshot taken when the request started.
static size_t csvChunk(uint32_t first, uint32_t total, uint8_t* buffer,
                       size_t maxLen, size_t index) {
  const size_t rows = (total < first) ? 0 : (size_t)(total - first + 1);
  const size_t end = CSV_HEADER_LEN + rows * CSV_ROW_LEN;
  size_t n = 0;

  while (n < maxLen && index < end) {
    if (index < CSV_HEADER_LEN) {
      size_t take = CSV_HEADER_LEN - index;
      if (take > maxLen - n) take = maxLen - n;
      memcpy(buffer + n, CSV_HEADER + index, take);
      n += take;
      index += take;
      continue;
    }

    const size_t offset = index - CSV_HEADER_LEN;
    const uint32_t seq = first + (uint32_t)(offset / CSV_ROW_LEN);
    const size_t col = offset % CSV_ROW_LEN;

    Sample s;
    if (!runLog.get(seq, s)) break;

    char row[CSV_ROW_LEN + 1];
    TelemetryRing::csvRow(seq, s, row, sizeof(row));
    size_t take = CSV_ROW_LEN - col;
    if (take > maxLen - n) take = maxLen - n;
    memcpy(buffer + n, row + col, take);
    n += take;
    index += take;
  }

  return n;
}

// The control task owns the supervisor and through it the HAL. After boot it is the only task that writes the servo, the motor or the I2C bus.
static void controlTask(void* arg) {
  Esp32Hal* h = static_cast<Esp32Hal*>(arg);

  Supervisor sup;
  sup.attach(h);
  sup.calibrate();

  // vTaskDelayUntil keeps the period honest, where a plain delay would add the tick's own cost to every period.
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    if (startRequest.exchange(false)) {
      const bool running = sup.phase == Phase::BACK_IN || sup.phase == Phase::CORRECT;
      if (running) {
        Serial.println("start ignored: run in progress");
      } else if (!sup.start()) {
        Serial.println("start ignored: not calibrated");
      } else {
        Serial.println("start");
      }
    }

    sup.update();
    runLog.record(sup.tel);

    // 10 Hz, every third 30 Hz tick, so a run reads on the serial line with no browser open. Units are spelled into the field names because a screenshot has no caption.
    static uint32_t tick = 0;
    if (++tick % 3 == 0) {
      Serial.printf("%7.2fs %-9s %-9s x=%+6.1f psi=%+6.1f rear=%6.1f gL=%6.1f gR=%6.1f servo=%5.1f cal=%d ok=%d strokes=%d\n",
                    sup.tel.t, phaseName(sup.tel.phase), strokeName(sup.tel.stroke),
                    sup.tel.x * 1000.0f, sup.tel.psi * 57.2958f,
                    sup.tel.rear * 1000.0f, sup.tel.gL * 1000.0f,
                    sup.tel.gR * 1000.0f, sup.tel.servo,
                    (int)sup.tel.calibrated, (int)sup.tel.valid, sup.tel.strokes);
    }

    xQueueOverwrite(telemetryQueue, &sup.tel);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// Join the configured network or fall back to an access point, so the page is reachable on a bench where that network is not there.
static void connectNetwork() {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nwifi joined, open http://%s/\n", WiFi.localIP().toString().c_str());
    return;
  }

  Serial.printf("\nwifi not joined in %us, starting the access point\n",
                (unsigned)(WIFI_TIMEOUT_MS / 1000));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("ap %s password %s, open http://%s/\n", AP_SSID, AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
}

// Bring the network and server up, then publish telemetry every 100 ms. The async server runs its own task once started, so this one only feeds it.
static void webTask(void* arg) {
  (void)arg;

  connectNetwork();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
  });

  // The page's button. This runs in the socket task and only sets the flag, so it never blocks on the control loop.
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* req) {
    startRequest.store(true);
    req->send(200, "application/json", "{\"start\":true}");
  });

  // The whole held run as CSV, which is also the page's backfill source. The bounds are snapshotted once and the filler derives each row from the request offset, so a request allocates no buffer.
  server.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    uint32_t first = 0;
    uint32_t total = 0;
    runLog.copySince(0, nullptr, 0, first, total);
    AsyncWebServerResponse* res = req->beginChunkedResponse(
        "text/csv",
        [first, total](uint8_t* buffer, size_t maxLen, size_t index) {
          return csvChunk(first, total, buffer, maxLen, index);
        });
    res->addHeader("Content-Disposition", "attachment; filename=\"log.csv\"");
    req->send(res);
  });

  server.addHandler(&socket);
  server.begin();

  TickType_t last = xTaskGetTickCount();
  Telemetry t;

  // One receive with no wait per period. If nothing has been published since the last look there is nothing to send, and the page keeps what it has.
  for (;;) {
    if (xQueueReceive(telemetryQueue, &t, 0) == pdTRUE) publishTelemetry(t, runLog.total());
    socket.cleanupClients();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("autoparking-car: boot");

  // The hardware comes up before either worker exists, since the bus and the address assignment are not safe to share with a task already reading them.
  if (!hal.begin()) Serial.println("one or more sensors did not answer");

  telemetryQueue = xQueueCreate(1, sizeof(Telemetry));
  if (telemetryQueue == nullptr) {
    Serial.println("no heap for the telemetry queue");
    for (;;) delay(1000);
  }

  if (xTaskCreatePinnedToCore(controlTask, "control", CONTROL_STACK, &hal,
                              CONTROL_PRIORITY, nullptr, CONTROL_CORE) != pdPASS) {
    Serial.println("control task not created");
  }
  if (xTaskCreatePinnedToCore(webTask, "web", WEB_STACK, nullptr, WEB_PRIORITY,
                              nullptr, WEB_CORE) != pdPASS) {
    Serial.println("web task not created");
  }

  Serial.println("control on core 1, web on core 0");
}

void loop() {
  // The work is in the two tasks. This says the runtime is alive and touches nothing the car depends on.
  delay(5000);
  Serial.printf("alive %lus heap %luB\n", (unsigned long)(millis() / 1000UL),
                (unsigned long)ESP.getFreeHeap());
}
