// the runtime. two freertos tasks, one per core: the control loop on core 1 and
// everything that talks to the network on core 0.
//
// they are split because they have nothing to do with each other. the control
// loop has to tick at 30 hz or the stroke profiles plan one manoeuvre and do
// another, and it touches the i2c bus, the servo and the motor. the web task
// waits on wifi association, formats json and hands it to the socket, none of
// which may delay a sensor read. on one core they would.
//
// the two tasks share one word going up and one struct going down. the
// supervisor object belongs to the control task and is never handed over: the
// queue carries a copy of the telemetry, so the web side cannot read a state
// that is halfway through an update.

#include <Arduino.h>
#include <WiFi.h>

#include <atomic>
#include <cmath>
#include <cstdio>

#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "dashboard_html.h"
#include "esp32_hal.h"
#include "supervisor.h"
#include "wifi_config.h"

// the control loop ticks at 30 hz, which is what Config::dt says. the task
// period has to match it: the control law measures its own dt, but the stroke
// profiles assume a tick, so a task running at another rate would plan against
// one period and run at another. ---
constexpr uint32_t CONTROL_PERIOD_MS = 33;
// end

// ten messages a second is enough for a phone to watch a manoeuvre that takes
// seconds, and it keeps the socket work far below the tick rate. the queue
// overwrite means a slower rate costs nothing but a skipped sample.
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

// how long the web task waits for the configured network before it gives up and
// brings up its own access point. ---
constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
// end

// core 1 is where the arduino loop runs and nothing else of ours does. core 0
// carries the wifi stack, so the web task belongs beside it rather than in the
// control loop's way. ---
constexpr BaseType_t CONTROL_CORE = 1;
constexpr BaseType_t WEB_CORE = 0;
// end

// stack sizes are bytes on the esp-idf port, not words. the control task holds
// the supervisor and reads three sensors per tick; the web task formats json
// and starts the server. ---
constexpr uint32_t CONTROL_STACK = 8192;
constexpr UBaseType_t CONTROL_PRIORITY = 5;
constexpr uint32_t WEB_STACK = 8192;
constexpr UBaseType_t WEB_PRIORITY = 2;
// end

// the handoff. the queue holds one item and the producer overwrites it, so the
// consumer can never fall behind and a slow socket can never delay the control
// loop: the web task either gets the newest tick or nothing at all. ---
static QueueHandle_t telemetryQueue = nullptr;
// end

// the only word that crosses the other way. it is a request rather than a
// command, because the control task still refuses it before the calibration
// and ignores it while a run is in progress.
static std::atomic<bool> startRequest{false};

static AsyncWebServer server(80);
static AsyncWebSocket socket("/ws");
static Esp32Hal hal;

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

// one json object per message, all integers. the page converts at the edge, so
// nothing here needs float formatting: t is ms, x, rear, travel, clearance are
// mm, psi, cap and peak are milliradians, servo is degrees. ---
static void publishTelemetry(const Telemetry& t) {
  if (socket.count() == 0) return;

  static char line[320];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"x\":%ld,\"psi\":%ld,\"rear\":%ld,\"servo\":%d,"
           "\"phase\":\"%s\",\"stroke\":\"%s\",\"strokes\":%d,"
           "\"cal\":%d,\"valid\":%d,\"travel\":%ld,\"cap\":%ld,"
           "\"clear\":%ld,\"peak\":%ld,"
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
           (int)t.rearFault, (int)t.travelFault, (int)t.outOfSpec);
  socket.textAll(line);
}
// end

// the control task owns the supervisor, and through it the hal: it is the only
// task that writes the servo, the motor or the i2c bus after boot. ---
static void controlTask(void* arg) {
  Esp32Hal* h = static_cast<Esp32Hal*>(arg);

  Supervisor sup;
  sup.attach(h);
  sup.calibrate();

  // vTaskDelayUntil keeps the period honest. a plain delay would add the time
  // the tick took to every period, and the supervisor integrates over measured
  // time while the profiles assume the nominal one. ---
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
    xQueueOverwrite(telemetryQueue, &sup.tel);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
  // end
}
// end

// join the configured network, or fall back to an access point of our own. the
// wait is bounded, and the fallback is the reason the page is reachable on a
// bench where the configured network is not there. ---
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
  // end
}
// end

// the web task brings the network and the server up and then publishes the
// newest telemetry every hundred milliseconds. the async server and the socket
// run their own task once started, so this one only feeds them. ---
static void webTask(void* arg) {
  (void)arg;

  connectNetwork();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
  });

  // the page's button. the handler runs in the socket task and only sets the
  // flag, so it never blocks on the control loop. ---
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* req) {
    startRequest.store(true);
    req->send(200, "application/json", "{\"start\":true}");
  });
  // end

  server.addHandler(&socket);
  server.begin();

  TickType_t last = xTaskGetTickCount();
  Telemetry t;

  for (;;) {
    // a receive with no wait: if the control task has not published since the
    // last look, there is nothing new to send and the page keeps what it has.
    if (xQueueReceive(telemetryQueue, &t, 0) == pdTRUE) publishTelemetry(t);
    socket.cleanupClients();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
  // end
}
// end

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("autoparking-car: boot");

  // the hardware comes up here, on the arduino task, before either worker
  // exists: the bus and the address assignment are not safe to share with a
  // task that is already reading them. ---
  if (!hal.begin()) Serial.println("one or more sensors did not answer");
  // end

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
  // the work is in the two tasks. this says the runtime is alive and touches
  // nothing the car depends on. ---
  delay(5000);
  Serial.printf("alive %lus heap %luB\n", (unsigned long)(millis() / 1000UL),
                (unsigned long)ESP.getFreeHeap());
  // end
}
