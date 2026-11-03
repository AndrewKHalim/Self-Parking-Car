// Runtime. Two FreeRTOS tasks, one core each: the 30 Hz control loop on core 1
// owns I2C, servo and motors, the network task on core 0 owns WiFi and JSON and
// may never delay a sensor read. The queue carries a copy of the telemetry, so
// the web side cannot read a half-written state.
#include <Arduino.h>
#include <WiFi.h>

#include <atomic>
#include <cstdio>

#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "actuators.h"
#include "config.h"
#include "dashboard_html.h"
#include "parking.h"
#include "sensors.h"

// Control shares core 1 with the Arduino loop task; the async TCP stack is
// pinned to core 0 by CONFIG_ASYNC_TCP_RUNNING_CORE in platformio.ini.
constexpr BaseType_t CONTROL_CORE = 1;
constexpr BaseType_t WEB_CORE = 0;

constexpr uint32_t CONTROL_STACK = 8192;  // bytes on the esp-idf port
constexpr UBaseType_t CONTROL_PRIORITY = 5;
constexpr uint32_t WEB_STACK = 8192;
constexpr UBaseType_t WEB_PRIORITY = 2;

// One slot, overwritten by the producer: the web side can never fall behind and
// a slow socket can never delay the control loop.
static QueueHandle_t telemetryQueue = nullptr;

// Requests crossing the other way. They only set flags; the control task reads
// them on its next tick, so no HTTP handler ever waits on the car.
static std::atomic<bool> startRequested{false};
static std::atomic<bool> stopRequested{false};
static std::atomic<uint16_t> pendingRearTargetMm{0};
static std::atomic<uint16_t> pendingAlignDiffMm{0};
// -1 leaves the cap alone; 0 is a legal value and means "no cap".
static std::atomic<int16_t> pendingBackoffMax{-1};

// A negative value means "unchanged", which is why the sentinel is -1: zero is
// a legal gain (that term switched off).
static std::atomic<float> pendingKp{-1.0f};
static std::atomic<float> pendingKi{-1.0f};
static std::atomic<float> pendingKd{-1.0f};

// The gains the machine is actually running, so the page reports what is in
// force rather than what the defaults happen to be.
static std::atomic<float> liveKp{pid_defaults::KP};
static std::atomic<float> liveKi{pid_defaults::KI};
static std::atomic<float> liveKd{pid_defaults::KD};

static AsyncWebServer server(80);
static AsyncWebSocket socket("/ws");

static SensorBank sensorBank;
static Actuators actuators;

// One JSON object per message, integers where the firmware has integers. The
// page prints the target and the gains so a run is self-describing.
static void publishTelemetry(const Telemetry& t) {
  if (socket.count() == 0) {
    return;
  }

  static char line[288];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"state\":\"%s\",\"gL\":%u,\"gR\":%u,\"diff\":%d,"
           "\"rear\":%u,\"rearTarget\":%u,\"alignDiff\":%u,\"backoffMax\":%u,"
           "\"servo\":%d,\"cycles\":%u,\"backoffs\":%u,\"fault\":\"%s\","
           "\"kp\":%.2f,\"ki\":%.3f,\"kd\":%.2f}",
           (unsigned long)t.t_ms, phaseName(t.phase), (unsigned)t.gL_mm,
           (unsigned)t.gR_mm, (int)t.diff_mm, (unsigned)t.rear_mm,
           (unsigned)t.rear_target_mm, (unsigned)t.align_diff_mm,
           (unsigned)t.backoff_max, (int)t.servo_deg, (unsigned)t.cycles,
           (unsigned)t.backoffs, t.fault, liveKp.load(), liveKi.load(),
           liveKd.load());
  socket.textAll(line);
}

// The control task is the only writer of the servo, the motors and the bus.
static void controlTask(void* arg) {
  (void)arg;

  ParkingMachine machine;
  uint32_t tick = 0;
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    const uint16_t wanted_target = pendingRearTargetMm.exchange(0);
    if (wanted_target != 0) {
      machine.setRearTargetMm(wanted_target);
    }

    const uint16_t wanted_align = pendingAlignDiffMm.exchange(0);
    if (wanted_align != 0) {
      machine.setAlignDiffMm(wanted_align);
    }

    const int16_t wanted_backoff = pendingBackoffMax.exchange(-1);
    if (wanted_backoff >= 0) {
      machine.setBackoffMax((uint8_t)wanted_backoff);
    }

    const float wanted_kp = pendingKp.exchange(-1.0f);
    const float wanted_ki = pendingKi.exchange(-1.0f);
    const float wanted_kd = pendingKd.exchange(-1.0f);
    if (wanted_kp >= 0.0f || wanted_ki >= 0.0f || wanted_kd >= 0.0f) {
      machine.setGains(wanted_kp, wanted_ki, wanted_kd);
      liveKp.store(machine.config().kp);
      liveKi.store(machine.config().ki);
      liveKd.store(machine.config().kd);
    }

    const SensorReading reading = sensorBank.read();
    const uint32_t now = millis();

    // Stop wins over start: an abort must not be swallowed by a queued start.
    if (stopRequested.exchange(false)) {
      machine.stop(actuators);
      Serial.println("stop");
    } else if (startRequested.exchange(false)) {
      machine.start(now, reading, actuators);
      Serial.printf("start rear=%umm target=%umm\n", (unsigned)reading.rear_mm,
                    (unsigned)machine.config().rear_target_mm);
    }

    const Telemetry t = machine.tick(now, reading, actuators);
    xQueueOverwrite(telemetryQueue, &t);

    // 10 Hz serial, so a run reads without a browser open. Units are spelled
    // into the field names because a terminal capture has no caption.
    if (++tick % 3 == 0) {
      Serial.printf(
          "t=%lu state=%s gL=%u gR=%u diff=%d rear=%u target=%u servo=%d "
          "wheel=%.1f cycles=%u backoffs=%u fault=%s\n",
          (unsigned long)t.t_ms, phaseName(t.phase), (unsigned)t.gL_mm,
          (unsigned)t.gR_mm, (int)t.diff_mm, (unsigned)t.rear_mm,
          (unsigned)t.rear_target_mm, (int)t.servo_deg, t.wheel_deg,
          (unsigned)t.cycles, (unsigned)t.backoffs, t.fault);
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(defaults::CONTROL_PERIOD_MS));
  }
}

// The car is its own access point, so the page is reachable with no router and
// no credentials to flash. Nothing is attempted in station mode.
static void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(net_cfg::AP_SSID, net_cfg::AP_PASSWORD);
  Serial.printf("ap %s password %s, open http://%s/\n", net_cfg::AP_SSID,
                net_cfg::AP_PASSWORD, WiFi.softAPIP().toString().c_str());
}

static void webTask(void* arg) {
  (void)arg;

  startAccessPoint();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
  });

  const auto handleStart = [](AsyncWebServerRequest* req) {
    startRequested.store(true);
    req->send(200, "application/json", "{\"start\":true}");
  };
  server.on("/start", HTTP_POST, handleStart);
  server.on("/start", HTTP_GET, handleStart);

  const auto handleStop = [](AsyncWebServerRequest* req) {
    stopRequested.store(true);
    req->send(200, "application/json", "{\"stop\":true}");
  };
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/stop", HTTP_GET, handleStop);

  // Both are clamped by the machine, so an out-of-range value from the page
  // cannot move the car into the wall or demand more than the servo delivers.
  const auto handleConfig = [](AsyncWebServerRequest* req) {
    if (req->hasParam("rear_target_mm")) {
      pendingRearTargetMm.store(
          (uint16_t)req->getParam("rear_target_mm")->value().toInt());
    }
    if (req->hasParam("align_diff_mm")) {
      pendingAlignDiffMm.store(
          (uint16_t)req->getParam("align_diff_mm")->value().toInt());
    }
    if (req->hasParam("backoff_max")) {
      pendingBackoffMax.store(
          (int16_t)req->getParam("backoff_max")->value().toInt());
    }
    if (req->hasParam("kp")) {
      pendingKp.store(req->getParam("kp")->value().toFloat());
    }
    if (req->hasParam("ki")) {
      pendingKi.store(req->getParam("ki")->value().toFloat());
    }
    if (req->hasParam("kd")) {
      pendingKd.store(req->getParam("kd")->value().toFloat());
    }
    req->send(200, "application/json", "{\"config\":true}");
  };
  server.on("/config", HTTP_POST, handleConfig);
  server.on("/config", HTTP_GET, handleConfig);

  server.addHandler(&socket);
  server.begin();

  TickType_t last = xTaskGetTickCount();
  Telemetry t;

  for (;;) {
    // Peek, not receive: the page gets the newest sample every period even if it
    // has not changed, and the producer is never blocked by this task.
    if (xQueuePeek(telemetryQueue, &t, 0) == pdTRUE) {
      publishTelemetry(t);
    }
    socket.cleanupClients();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(defaults::TELEMETRY_PERIOD_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("autoparking-car: boot");

  // The bus and the address assignment come up before either task exists, since
  // sharing them with a task already reading is not safe.
  if (!sensorBank.begin()) {
    Serial.println("one or more sensors did not answer");
  }
  actuators.begin();

  Serial.printf(
      "target=%umm tol=%umm align=%umm settle=%ums forward=%ums backoff=%ums "
      "backoff_max=%u servo_per_wheel=%.2f max_wheel=%.1f kp=%.2f ki=%.3f "
      "kd=%.2f\n",
      (unsigned)defaults::REAR_TARGET_MM, (unsigned)defaults::REAR_TOLERANCE_MM,
      (unsigned)defaults::ALIGN_DIFF_MM, (unsigned)defaults::SETTLE_MS,
      (unsigned)defaults::FORWARD_MOVE_MS, (unsigned)defaults::BACKOFF_MOVE_MS,
      (unsigned)defaults::BACKOFF_MAX, steering::SERVO_PER_WHEEL_DEG,
      steering::MAX_WHEEL_DEG, pid_defaults::KP, pid_defaults::KI,
      pid_defaults::KD);

  telemetryQueue = xQueueCreate(1, sizeof(Telemetry));
  if (telemetryQueue == nullptr) {
    Serial.println("no heap for the telemetry queue");
    for (;;) {
      delay(1000);
    }
  }

  if (xTaskCreatePinnedToCore(controlTask, "control", CONTROL_STACK, nullptr,
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
  // The work lives in the two tasks. This only says the runtime is alive.
  delay(5000);
  Serial.printf("alive %lus heap %luB\n", (unsigned long)(millis() / 1000UL),
                (unsigned long)ESP.getFreeHeap());
}
