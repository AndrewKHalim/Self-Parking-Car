// Straight reverse, rear stop, side guard. There is no steering law.
//
// The two side faces give one number, gL - gR, for two unknowns, the lateral
// offset and the yaw. At 5 degrees of yaw the heading term alone is 18.6 mm of
// false offset, so a controller on that mixture has no unique set point. main
// tried anyway and worked sometimes. This firmware keeps the wheels centred,
// reverses in a straight line, stops on the rear sensor and refuses to move if
// the side pair says the car is already against a wall.
//
// The states are IDLE, CHECK, REVERSE, BRAKE, PARKED and FAULT. One loop at
// delay(25). The web server runs its own task on core 0 through AsyncTCP.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#include "config.h"
#include "dashboard_html.h"
#include "drive.h"
#include "sensors.h"
#include "wifi_config.h"

Sensors sensors;
Drive motors;
Servo steering;

enum State : uint8_t {
  STATE_IDLE = 0,
  STATE_CHECK,
  STATE_REVERSE,
  STATE_BRAKE,
  STATE_PARKED,
  STATE_FAULT
};

enum Fault : uint8_t {
  FAULT_NONE = 0,
  FAULT_REAR,
  FAULT_SIDE,
  FAULT_TIMEOUT
};

State state_ = STATE_IDLE;
Fault fault_ = FAULT_NONE;
bool ready_ = false;
bool startAfterCheck_ = false;
unsigned long stateStart_ = 0;
unsigned long lastLoop_ = 0;
int rearInvalid_ = 0;

// The CHECK state samples one frame per loop and takes the median of five. One
// frame per loop is enough because the part updates every 33 ms and the loop
// runs at 25 ms, so five frames cover about four rangings.
uint16_t framesL_[parking::CHECK_FRAMES];
uint16_t framesR_[parking::CHECK_FRAMES];
uint16_t framesRear_[parking::CHECK_FRAMES];
int frameCount_ = 0;

AsyncWebServer server(80);
AsyncWebSocket socket("/ws");
std::atomic<bool> startRequest{false};

const char* stateName(State s) {
  switch (s) {
  case STATE_CHECK:
    return "check";
  case STATE_REVERSE:
    return "reverse";
  case STATE_BRAKE:
    return "brake";
  case STATE_PARKED:
    return "parked";
  case STATE_FAULT:
    return "fault";
  default:
    return "idle";
  }
}

const char* faultName(Fault f) {
  switch (f) {
  case FAULT_REAR:
    return "rear";
  case FAULT_SIDE:
    return "side";
  case FAULT_TIMEOUT:
    return "timeout";
  default:
    return "none";
  }
}

void enter(State next) {
  state_ = next;
  stateStart_ = millis();

  // ready means the page may offer a start. It is true only where a start is
  // accepted, so the button is dark for the whole of a run and in a fault.
  ready_ = (next == STATE_IDLE || next == STATE_PARKED);

  if (next == STATE_CHECK) {
    frameCount_ = 0;
    fault_ = FAULT_NONE;
  }
  if (next == STATE_REVERSE) rearInvalid_ = 0;
}

uint16_t median5(uint16_t* v) {
  for (int i = 1; i < 5; ++i) {
    const uint16_t key = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      --j;
    }
    v[j + 1] = key;
  }
  return v[2];
}

// One line per part and then a verdict, so a flashed board says which sensor is
// dead before anything moves. The raw value is the code the part answered with,
// 8190 for no target and 65535 for a timeout or a part that never came up.
void printSelfTest() {
  for (int i = 0; i < Sensors::COUNT; ++i) {
    Serial.printf("part %-5s xshut=%2d addr=0x%02X init=%s raw=%u mm\n",
                  Sensors::name(i), Sensors::xshutPin(i), Sensors::address(i),
                  sensors.initOk(i) ? "ok" : "fail", sensors.bootRaw(i));
  }

  const uint16_t rear = sensors.bootRaw(0);
  const uint16_t left = sensors.bootRaw(2);
  const uint16_t right = sensors.bootRaw(1);
  const bool rearVisible =
      Sensors::fresh(rear) && rear >= sensor::REAR_VALID_MIN_MM &&
      rear <= sensor::REAR_VALID_MAX_MM;
  const bool sidesUsable = Sensors::fresh(left) && Sensors::fresh(right) &&
                           left >= sensor::SIDE_GUARD_MM &&
                           right >= sensor::SIDE_GUARD_MM;
  Serial.printf("verdict rear=%s sides=%s\n",
                rearVisible ? "visible" : "not visible",
                sidesUsable ? "usable" : "not usable");
}

void printBootConfig() {
  Serial.printf("pins i2c=%d/%d xshut=%d/%d/%d servo=%d motor=%d/%d/%d/%d\n",
                board::PIN_I2C_SDA, board::PIN_I2C_SCL, board::PIN_XSHUT_L,
                board::PIN_XSHUT_R, board::PIN_XSHUT_REAR, board::PIN_SERVO,
                board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                board::PIN_MOTOR_B1, board::PIN_MOTOR_B2);
  Serial.printf("drive pwm=%luHz/%ubit duty=%u/%u servo=%d..%d centre=%d\n",
                (unsigned long)drive::PWM_HZ, (unsigned)drive::PWM_BITS,
                (unsigned)drive::DUTY_REVERSE, (unsigned)drive::DUTY_FORWARD,
                servo::MIN_DEG, servo::MAX_DEG, servo::CENTRE_DEG);
  Serial.printf("times loop=%lums check=%d frames reverseMax=%lums "
                "brake=%lums stop=%.0fmm guard=%umm\n",
                timing::LOOP_MS, (int)parking::CHECK_FRAMES,
                parking::REVERSE_MAX_MS, parking::BRAKE_MS,
                parking::REAR_STOP_MM, sensor::SIDE_GUARD_MM);
}

// One JSON object, every field an integer or a name. The gaps and the rear are
// raw millimetres, so 8190 and 65535 reach the page and it can say none.
void publish(const Readings& r, unsigned long now) {
  if (socket.count() == 0) return;

  static char line[224];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"state\":\"%s\",\"fault\":\"%s\",\"gL\":%u,\"gR\":%u,"
           "\"rear\":%u,\"servo\":%d,\"ready\":%d}",
           now, stateName(state_), faultName(fault_), r.gL, r.gR, r.rear,
           servo::CENTRE_DEG, (int)ready_);
  socket.textAll(line);
}

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

void setup() {
  Serial.begin(115200);
  const unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
    delay(10);
  }
  delay(200);

  Serial.println();
  Serial.println("autoparking-car: straight reverse, rear stop, side guard");
  printBootConfig();

  Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);
  Wire.setClock(400000);

  steering.attach(board::PIN_SERVO);
  steering.write(servo::CENTRE_DEG);

  motors.begin();

  sensors.begin();
  delay(timing::SENSOR_SETTLE_MS);
  sensors.probe();
  printSelfTest();

  connectNetwork();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/html", DASHBOARD_HTML);
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* req) {
    startRequest.store(true);
    req->send(200, "application/json", "{\"start\":true}");
  });

  server.addHandler(&socket);
  server.begin();

  // The boot check validates the sensors before the page offers a start. It
  // ends in IDLE rather than REVERSE, so only a press moves the car.
  startAfterCheck_ = false;
  enter(STATE_CHECK);
  lastLoop_ = millis();
  Serial.println("checking sensors");
}

void loop() {
  const unsigned long now = millis();
  lastLoop_ = now;

  const Readings r = sensors.read();

  if (startRequest.exchange(false)) {
    if (state_ == STATE_IDLE || state_ == STATE_PARKED) {
      startAfterCheck_ = true;
      enter(STATE_CHECK);
    }
  }

  switch (state_) {
  case STATE_IDLE:
    motors.brake();
    steering.write(servo::CENTRE_DEG);
    break;

  case STATE_CHECK: {
    if (frameCount_ < parking::CHECK_FRAMES) {
      framesL_[frameCount_] = r.gL;
      framesR_[frameCount_] = r.gR;
      framesRear_[frameCount_] = r.rear;
      ++frameCount_;
    }
    motors.brake();
    steering.write(servo::CENTRE_DEG);
    if (frameCount_ < parking::CHECK_FRAMES) break;

    const uint16_t mL = median5(framesL_);
    const uint16_t mR = median5(framesR_);
    const uint16_t mRear = median5(framesRear_);
    const bool rearOk = Sensors::fresh(mRear) &&
                        mRear >= sensor::REAR_VALID_MIN_MM &&
                        mRear <= sensor::REAR_VALID_MAX_MM;
    const bool sidesOk = Sensors::fresh(mL) && Sensors::fresh(mR) &&
                         mL >= sensor::SIDE_GUARD_MM &&
                         mR >= sensor::SIDE_GUARD_MM;
    if (!rearOk || !sidesOk) {
      fault_ = rearOk ? FAULT_SIDE : FAULT_REAR;
      Serial.printf("check failed rear=%u left=%u right=%u fault=%s\n", mRear,
                    mL, mR, faultName(fault_));
      enter(STATE_FAULT);
      break;
    }
    if (startAfterCheck_) {
      enter(STATE_REVERSE);
    } else {
      enter(STATE_IDLE);
    }
    break;
  }

  case STATE_REVERSE: {
    // The stop is a fresh rear reading at or under 90 mm. A fresh 0 to 29 mm is
    // inside that window and means the face is at the wall, so it stops too.
    const bool freshRear = Sensors::fresh(r.rear);
    const bool sideClose =
        (Sensors::fresh(r.gL) && r.gL < sensor::SIDE_GUARD_MM) ||
        (Sensors::fresh(r.gR) && r.gR < sensor::SIDE_GUARD_MM);

    if (freshRear &&
        (r.rear <= parking::REAR_STOP_MM || Sensors::atWall(r.rear))) {
      enter(STATE_BRAKE);
      break;
    }
    if (sideClose) {
      fault_ = FAULT_SIDE;
      enter(STATE_BRAKE);
      break;
    }
    if (!freshRear) {
      if (++rearInvalid_ >= parking::REAR_INVALID_LIMIT) {
        fault_ = FAULT_REAR;
        enter(STATE_BRAKE);
        break;
      }
    } else {
      rearInvalid_ = 0;
    }
    // The cap is a give-up, not a wall. It names timeout so a run that ran out
    // of time cannot be mistaken for one that reached the rear limit ---
    if (now - stateStart_ >= parking::REVERSE_MAX_MS) {
      fault_ = FAULT_TIMEOUT;
      enter(STATE_BRAKE);
      break;
    }
    // end

    steering.write(servo::CENTRE_DEG);
    motors.reverse(drive::DUTY_REVERSE);
    break;
  }

  case STATE_BRAKE:
    motors.brake();
    steering.write(servo::CENTRE_DEG);
    if (now - stateStart_ >= parking::BRAKE_MS) {
      enter(fault_ == FAULT_NONE ? STATE_PARKED : STATE_FAULT);
    }
    break;

  case STATE_PARKED:
    motors.brake();
    steering.write(servo::CENTRE_DEG);
    break;

  case STATE_FAULT:
    motors.brake();
    steering.write(servo::CENTRE_DEG);
    break;
  }

  static unsigned long lastReport = 0;
  const unsigned long t = millis();
  if (t - lastReport >= timing::LOG_MS) {
    lastReport = t;
    Serial.printf("t=%lu state=%s gL=%u gR=%u rear=%u servo=%d ready=%d "
                  "fault=%s\n",
                  t, stateName(state_), r.gL, r.gR, r.rear,
                  servo::CENTRE_DEG, (int)ready_, faultName(fault_));
    publish(r, t);
    socket.cleanupClients();
  }

  delay(timing::LOOP_MS);
}
