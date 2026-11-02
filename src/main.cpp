// The whole firmware. One loop at delay(25), one PID on the lateral error, one
// small state machine, one serial line at 10 Hz and a dashboard. The network
// side is the AsyncWebServer, which runs its own task on core 0 through
// AsyncTCP. Nothing here owns a control task, a scheduler or a HAL.
//
// The maneuver is main's. Reverse until the rear reading reaches 76 mm. If the
// gap difference is worth correcting, pull forward once with the wheels turned
// 25 degrees against the larger gap, pause, and reverse again. Three
// corrections at most, then park.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include <atomic>
#include <cmath>
#include <cstdio>

#include "DistanceSensorManager.h"
#include "MotorController.h"
#include "PIDController.h"
#include "ParkingConfig.h"
#include "dashboard_html.h"
#include "wifi_config.h"

DistanceSensorManager sensors;
PIDController steerPid(pid::KP, pid::KI, pid::KD);
MotorController motor;
Servo steering;

enum Phase : uint8_t {
  PHASE_IDLE = 0,
  PHASE_REVERSE,
  PHASE_PULL,
  PHASE_PARKED
};

enum Fault : uint8_t {
  FAULT_NONE = 0,
  FAULT_REAR,
  FAULT_TIMEOUT
};

Phase phase = PHASE_IDLE;
Fault fault = FAULT_NONE;

// phaseStart times the current phase, legStart times the current reverse leg.
// The 4 s cap is per reverse leg, so a pull forward gets a fresh clock.
unsigned long phaseStart = 0;
unsigned long legStart = 0;
unsigned long lastLoop = 0;
int pulls = 0;
float lastGapMm = 0.0f;
float pullGapMm = 0.0f;

AsyncWebServer server(80);
AsyncWebSocket socket("/ws");

// The one word the page sets. The handler runs in the AsyncTCP task on core 0
// and only flips this flag, so it never waits on the control loop.
std::atomic<bool> startRequest{false};

const char* phaseName(Phase p) {
  switch (p) {
  case PHASE_REVERSE:
    return "reverse";
  case PHASE_PULL:
    return "pull";
  case PHASE_PARKED:
    return "parked";
  default:
    return "idle";
  }
}

const char* faultName(Fault f) {
  switch (f) {
  case FAULT_REAR:
    return "rear";
  case FAULT_TIMEOUT:
    return "timeout";
  default:
    return "none";
  }
}

void enter(Phase next) {
  phase = next;
  phaseStart = millis();
  if (next == PHASE_REVERSE) legStart = phaseStart;
}

void beginRun() {
  pulls = 0;
  fault = FAULT_NONE;
  lastGapMm = 0.0f;
  pullGapMm = 0.0f;
  steerPid.reset();
  enter(PHASE_REVERSE);
}

// One JSON object per message, every field an integer. t is ms, the gaps and
// the rear are mm, servo is degrees. The l, r and b flags say whether that
// channel had a fresh reading, because an invalid sensor keeps its last value
// and the page must not show it as live.
void publish(const SensorReadings& r, uint32_t now, int servoDeg) {
  if (socket.count() == 0) return;

  static char line[288];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"phase\":\"%s\",\"fault\":\"%s\",\"gL\":%ld,"
           "\"gR\":%ld,\"rear\":%ld,\"servo\":%d,\"corr\":%d,"
           "\"l\":%d,\"r\":%d,\"b\":%d}",
           (unsigned long)now, phaseName(phase), faultName(fault),
           lroundf(r.gL), lroundf(r.gR), lroundf(r.gRear), servoDeg, pulls,
           (int)r.leftValid, (int)r.rightValid, (int)r.rearValid);
  socket.textAll(line);
}

// Join the configured network or fall back to an access point, so the page is
// reachable on a bench where that network is not there.
void connectNetwork() {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long started = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - started < timing::WIFI_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nwifi joined, open http://%s/\n",
                  WiFi.localIP().toString().c_str());
    return;
  }

  Serial.printf("\nno wifi in %lus, starting the access point\n",
                (unsigned long)(timing::WIFI_TIMEOUT_MS / 1000));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("ap %s password %s, open http://%s/\n", AP_SSID, AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
}

// One boot line names every value that decides a move, then one raw reading
// from each sensor, so a dead part is obvious before anything moves.
void printBootLine(const SensorReadings& boot, bool sensorsOk) {
  Serial.printf("pins sda=%d scl=%d xshutL=%d xshutR=%d xshutRear=%d "
                "servo=%d in1=%d in2=%d b1=%d b2=%d\n",
                board::PIN_I2C_SDA, board::PIN_I2C_SCL, board::PIN_XSHUT_L,
                board::PIN_XSHUT_R, board::PIN_XSHUT_REAR, board::PIN_SERVO,
                board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                board::PIN_MOTOR_B1, board::PIN_MOTOR_B2);
  Serial.printf("addr left=0x%02X right=0x%02X rear=0x%02X budget=%lums "
                "period=%lums\n",
                board::ADDR_LEFT, board::ADDR_RIGHT, board::ADDR_REAR,
                (unsigned long)(sensor::BUDGET_US / 1000),
                (unsigned long)sensor::PERIOD_MS);
  Serial.printf("drive pwm=%luHz/%ubit reverse=%u forward=%u\n",
                (unsigned long)drive::PWM_HZ, (unsigned)drive::PWM_BITS,
                (unsigned)drive::DUTY_REVERSE, (unsigned)drive::DUTY_FORWARD);
  Serial.printf("times loop=%lums log=%lums pull=%lums pause=%lums "
                "reverseMax=%lums\n",
                timing::LOOP_MS, timing::LOG_MS, parking::PULL_FORWARD_MS,
                parking::PAUSE_MS, parking::REVERSE_MAX_MS);
  Serial.printf("pid kp=%.2f ki=%.2f kd=%.2f deadband=%.0fmm stop=%.0fmm "
                "gap=%.0fmm steer=%.0fdeg pulls=%d\n",
                pid::KP, pid::KI, pid::KD, pid::DEADBAND_CM * 10.0f,
                parking::REAR_STOP_MM, parking::PULL_GAP_MM,
                parking::PULL_STEER_DEG, parking::MAX_PULLS);
  Serial.printf("raw left=%u right=%u rear=%u mm, sensorInit=%s\n",
                boot.rawL, boot.rawR, boot.rawRear,
                sensorsOk ? "ok" : "failed");
}

void setup() {
  Serial.begin(115200);
  const unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
    delay(10);
  }
  delay(200);

  Serial.println();
  Serial.println("autoparking-car: simple");
  Serial.println("one loop, one PID, one maneuver, a dashboard");

  Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);
  Wire.setClock(400000);

  // The library default pulse map is 544 to 2400 us over 0 to 180 degrees ---
  steering.attach(board::PIN_SERVO);
  steering.write(servo::CENTRE_DEG);
  // end

  motor.begin();

  const bool sensorsOk = sensors.begin();
  delay(timing::SENSOR_SETTLE_MS);
  const SensorReadings boot = sensors.read();
  printBootLine(boot, sensorsOk);

  connectNetwork();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* req) {
    startRequest.store(true);
    req->send(200, "application/json", "{\"start\":true}");
  });

  socket.onEvent([](AsyncWebSocket* srv, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      Serial.printf("ws client %lu connected\n",
                    (unsigned long)client->id());
    }
  });

  server.addHandler(&socket);
  server.begin();

  enter(PHASE_IDLE);
  lastLoop = millis();
  Serial.println("idle, press start on the page");
}

void loop() {
  const unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  if (dt <= 0.001f) dt = timing::LOOP_MS / 1000.0f;
  lastLoop = now;

  const SensorReadings r = sensors.read();

  // main's signal is the gap difference. A side dropout holds the last valid
  // gap rather than inventing a nominal clearance nobody measured ---
  if (r.leftValid && r.rightValid) lastGapMm = r.gL - r.gR;
  const float gapMm = lastGapMm;
  // end

  // A start is accepted from idle or parked. A press while the car is moving
  // is dropped, so the button cannot restart a run in progress ---
  if (startRequest.exchange(false)) {
    if (phase == PHASE_IDLE || phase == PHASE_PARKED) beginRun();
  }
  // end

  int servoAngle = servo::CENTRE_DEG;

  switch (phase) {
  case PHASE_IDLE:
    motor.stop();
    steering.write(servo::CENTRE_DEG);
    break;

  case PHASE_REVERSE: {
    // A held rear reading cannot fire the stop, only a fresh one under 76 mm ---
    const bool rearArrived = r.rearValid && r.gRear <= parking::REAR_STOP_MM;
    // end

    if (rearArrived) {
      motor.stop();
      steering.write(servo::CENTRE_DEG);
      servoAngle = servo::CENTRE_DEG;

      // A gap worth correcting while the car is close buys one pull forward.
      // Once three are spent the car parks rather than hunting for a wall.
      const bool misaligned = fabsf(gapMm) > parking::PULL_GAP_MM;
      const bool stillClose = r.gRear < parking::PULL_REAR_MAX_MM;
      // end

      if (misaligned && stillClose && pulls < parking::MAX_PULLS) {
        ++pulls;
        pullGapMm = gapMm;
        enter(PHASE_PULL);
      } else {
        enter(PHASE_PARKED);
      }
      break;
    }

    // The cap is a safety, not a distance. A dead rear part names its raw value ---
    if (now - legStart >= parking::REVERSE_MAX_MS) {
      motor.stop();
      steering.write(servo::CENTRE_DEG);
      servoAngle = servo::CENTRE_DEG;
      if (!r.rearValid) {
        fault = FAULT_REAR;
        Serial.printf("fault=rear raw=%u mm, stopped after %lu ms\n", r.rawRear,
                      now - legStart);
      } else {
        fault = FAULT_TIMEOUT;
        Serial.printf("fault=timeout rear=%.0f mm, stopped after %lu ms\n",
                      r.gRear, now - legStart);
      }
      enter(PHASE_PARKED);
      break;
    }
    // end

    const float offset = steerPid.compute(gapMm / 10.0f, dt);
    servoAngle = PIDController::toServoAngle(offset);
    steering.write(servoAngle);
    motor.reverse(drive::DUTY_REVERSE);
    break;
  }

  case PHASE_PULL: {
    // main's counter-steer, 25 degrees opposite the larger gap ---
    const float counterDeg =
        (pullGapMm > 0.0f) ? -parking::PULL_STEER_DEG : parking::PULL_STEER_DEG;
    servoAngle = PIDController::toServoAngle(counterDeg);
    steering.write(servoAngle);
    // end
    motor.forward(drive::DUTY_FORWARD);

    // The pull ends on time or as soon as the rear has room again. The pause is
    // main's, so the gear change does not land while the car rolls forward ---
    const bool timedOut = now - phaseStart >= parking::PULL_FORWARD_MS;
    const bool cleared = r.rearValid && r.gRear > parking::PULL_EXIT_MM;
    // end

    if (timedOut || cleared) {
      motor.stop();
      steering.write(servo::CENTRE_DEG);
      servoAngle = servo::CENTRE_DEG;
      steerPid.reset();
      delay(parking::PAUSE_MS);
      enter(PHASE_REVERSE);
    }
    break;
  }

  case PHASE_PARKED:
    motor.stop();
    steering.write(servo::CENTRE_DEG);
    servoAngle = servo::CENTRE_DEG;
    break;
  }

  // One line at 10 Hz, and the same snapshot to the page. Units are in the
  // field names because a serial capture has no caption ---
  static unsigned long lastReport = 0;
  const unsigned long t = millis();
  if (t - lastReport >= timing::LOG_MS) {
    lastReport = t;
    Serial.printf("%8lu phase=%s gL=%.0f gR=%.0f rear=%.0f servo=%d corr=%d "
                  "fault=%s\n",
                  t, phaseName(phase), r.gL, r.gR, r.gRear, servoAngle, pulls,
                  faultName(fault));
    publish(r, t, servoAngle);
    socket.cleanupClients();
  }
  // end

  delay(timing::LOOP_MS);
}
