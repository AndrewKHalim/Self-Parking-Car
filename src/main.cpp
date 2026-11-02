// Reverse into the bay under a cascade PID, stop on the rear sensor.
//
// The two side faces sit at different stations along the car, so the pair
// separates lateral offset from heading once the bay width is measured at the
// start of the run. The derivation is in control.h. The outer loop on offset
// asks for a heading, the inner loop on heading asks for a road wheel angle,
// and the linkage turns that into a servo command. Reversing steering is
// unstable, so the heading loop is the fast one.
//
// The states are IDLE, CHECK, CALIBRATE, REVERSE, BRAKE, PARKED and FAULT. One
// loop at delay(25). The web server runs its own task on core 0 through
// AsyncTCP.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#include "config.h"
#include "control.h"
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
  STATE_CALIBRATE,
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

// Every channel is filtered once per loop and everything downstream reads the
// filtered number. An invalid frame holds the last good value, so a single
// dropped reading does not jerk the pose.
// end
Ema filterL_;
Ema filterR_;
Ema filterRear_;
float gLf_ = sensor::NO_TARGET_MM;
float gRf_ = sensor::NO_TARGET_MM;
float rearF_ = sensor::NO_TARGET_MM;

// The bay width in metres, measured once per run, and the pose that follows
// from it. pose_ stays zero until a run has calibrated.
// end
Cascade steer_;
Steer steerOut_ = {0.0f, servo::CENTRE_DEG};
Pose pose_ = {0.0f, 0.0f};
float s0_ = 0.0f;
bool calibrated_ = false;
int calCount_ = 0;
float calSum_ = 0.0f;
float calDiff_ = 0.0f;
unsigned long lastCalSample_ = 0;
unsigned long lastSteer_ = 0;

AsyncWebServer server(80);
AsyncWebSocket socket("/ws");
std::atomic<bool> startRequest{false};

const char* stateName(State s) {
  switch (s) {
  case STATE_CHECK:
    return "check";
  case STATE_CALIBRATE:
    return "calibrate";
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
  if (next == STATE_CALIBRATE) {
    calCount_ = 0;
    calSum_ = 0.0f;
    calDiff_ = 0.0f;
    lastCalSample_ = millis();
    calibrated_ = false;
    steer_.reset();
  }
  if (next == STATE_REVERSE) {
    rearInvalid_ = 0;
    lastSteer_ = millis();
    steer_.seed(pose_.psi);
  }
  if (next == STATE_IDLE) steer_.reset();

  // Outside REVERSE the wheels are written to centre, so the reported command
  // has to say centre too. Only REVERSE produces a steering demand.
  // end
  if (next != STATE_REVERSE) {
    steerOut_.psiRef = 0.0f;
    steerOut_.servoDeg = servo::CENTRE_DEG;
  }
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
  Serial.printf("control steer=%d Kx=%.2f Ki=%.2f Kp=%.2f Kd=%.2f "
                "psiRefMax=%.2f deltaMax=%.2f deadband=%.0fmm/%.3frad\n",
                (int)control::STEER_ENABLE, control::KX, control::KI,
                control::KP, control::KD, control::PSI_REF_MAX,
                control::DELTA_MAX, control::X_DEADBAND_M * 1000.0f,
                control::PSI_DEADBAND_RAD);
  Serial.printf("geometry dL=%.3f dR=%.3f sum=%.3f diff=%.3f "
                "perWheel=%.1f dir=%d tau=%.3fs\n",
                car::d_L, car::d_R, car::D_SUM, car::D_DIFF,
                control::PER_WHEEL, control::DIR, control::PSI_TAU_S);
}

// One JSON object, every field an integer or a name. The gaps and the rear are
// filtered millimetres, so 8190 and 65535 reach the page and it can say none.
// rearRaw is the frame the brake compares, sent beside the filtered rear so the
// page can show both and the lag is visible.
// x is millimetres, psi and psiRef are milliradians, and the servo is degrees.
// end
void publish(unsigned long now, uint16_t rearRaw) {
  if (socket.count() == 0) return;

  static char line[320];
  snprintf(line, sizeof(line),
           "{\"t\":%lu,\"state\":\"%s\",\"fault\":\"%s\",\"x\":%ld,"
           "\"psi\":%ld,\"psiRef\":%ld,\"servo\":%d,\"gL\":%d,\"gR\":%d,"
           "\"rear\":%d,\"rearRaw\":%u,\"ready\":%d}",
           now, stateName(state_), faultName(fault_),
           (long)lroundf(pose_.x * 1000.0f),
           (long)lroundf(pose_.psi * 1000.0f),
           (long)lroundf(steerOut_.psiRef * 1000.0f), steerOut_.servoDeg,
           (int)lroundf(gLf_), (int)lroundf(gRf_), (int)lroundf(rearF_), rearRaw,
           (int)ready_);
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
  Serial.println("autoparking-car: cascade PID reverse, rear stop, side guard");
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

  // One filter pass for the whole loop, before any state reads a number. An
  // invalid frame holds the last good value, so a dropped reading does not jerk
  // the pose, and the freshness tests below still run on the raw frame.
  // end
  if (Sensors::fresh(r.gL)) gLf_ = filterL_.step(r.gL, sensor::EPA);
  if (Sensors::fresh(r.gR)) gRf_ = filterR_.step(r.gR, sensor::EPA);
  if (Sensors::fresh(r.rear)) rearF_ = filterRear_.step(r.rear, sensor::EPA);

  // The pose exists only after a run has measured the bay width. Until then
  // the page shows zero rather than a heading against an unknown S0.
  // end
  if (calibrated_) {
    pose_ = poseFromGaps(gLf_ / 1000.0f, gRf_ / 1000.0f, s0_);
  }

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
      enter(STATE_CALIBRATE);
    } else {
      enter(STATE_IDLE);
    }
    break;
  }

  case STATE_CALIBRATE: {
    motors.brake();
    steering.write(servo::CENTRE_DEG);

    // One sample per sensor period. The loop is faster than the part, and a
    // repeated frame would weight the average towards a stuck reading.
    // end
    const bool pairOk = Sensors::fresh(r.gL) && Sensors::fresh(r.gR) &&
                        r.gL >= sensor::SIDE_GUARD_MM &&
                        r.gR >= sensor::SIDE_GUARD_MM;
    if (pairOk && now - lastCalSample_ >= sensor::PERIOD_MS) {
      calSum_ += gLf_ + gRf_;
      calDiff_ += gLf_ - gRf_;
      ++calCount_;
      lastCalSample_ = now;
    }

    if (calCount_ >= parking::CALIBRATE_FRAMES) {
      // A run only starts reversing on a rear frame that came back this
      // instant. The gate is freshness alone, because a rear channel that
      // reads no target or a timeout has nothing to stop the car with.
      // end
      if (!Sensors::fresh(r.rear)) {
        fault_ = FAULT_REAR;
        Serial.printf("reverse gate rear=%u fault=%s\n", r.rear,
                      faultName(fault_));
        enter(STATE_FAULT);
        break;
      }
      s0_ = calSum_ / (calCount_ * 1000.0f);
      // x0 is the offset the car entered with. It is printed for the owner and
      // is not subtracted from anything, because the loop has to centre the
      // car and not hold it where it started.
      // end
      const float x0 = 0.5f * calDiff_ / calCount_;
      calibrated_ = true;
      pose_ = poseFromGaps(gLf_ / 1000.0f, gRf_ / 1000.0f, s0_);
      Serial.printf("calibrate frames=%d S0=%.1fmm x0=%.1fmm\n", calCount_,
                    s0_ * 1000.0f, x0);
      enter(STATE_REVERSE);
      break;
    }
    if (now - stateStart_ >= parking::CALIBRATE_MAX_MS) {
      fault_ = FAULT_SIDE;
      Serial.printf("calibrate failed frames=%d\n", calCount_);
      enter(STATE_FAULT);
      break;
    }
    break;
  }

  case STATE_REVERSE: {
    // The stop is the raw rear frame at or under 90 mm. The filter trails two
    // or three samples at EPA 0.35 and three blocking reads make a loop about
    // 100 ms, so the filtered value is still near 90 mm when the tail is at the
    // wall. The filtered value stays as a secondary, never as the only test.
    // end
    const bool freshRear = Sensors::fresh(r.rear);
    const bool sideClose =
        (Sensors::fresh(r.gL) && r.gL < sensor::SIDE_GUARD_MM) ||
        (Sensors::fresh(r.gR) && r.gR < sensor::SIDE_GUARD_MM);

    if (freshRear && (r.rear <= parking::REAR_STOP_MM ||
                      rearF_ <= parking::REAR_STOP_MM)) {
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
    // of time cannot be mistaken for one that reached the rear limit.
    if (now - stateStart_ >= parking::REVERSE_MAX_MS) {
      fault_ = FAULT_TIMEOUT;
      enter(STATE_BRAKE);
      break;
    }

    // The measured dt feeds the derivative, so a slow tick is not read as a
    // fast heading change. The clamp keeps the first tick after entry, and a
    // stalled loop, from producing a nonsense rate.
    // end
    const unsigned long dtMs = now - lastSteer_;
    lastSteer_ = now;
    float dt = dtMs / 1000.0f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;

    steerOut_ = steer_.step(pose_.x, pose_.psi, dt);
    steering.write(steerOut_.servoDeg);
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
    Serial.printf("t=%lu state=%s x=%.1fmm psi=%.4frad psiRef=%.4frad "
                  "servo=%d gL=%.1f gR=%.1f rear=%.1f raw=%u ready=%d "
                  "fault=%s\n",
                  t, stateName(state_), pose_.x * 1000.0f, pose_.psi,
                  steerOut_.psiRef, steerOut_.servoDeg, gLf_, gRf_, rearF_,
                  r.rear, (int)ready_, faultName(fault_));
    publish(t, r.rear);
    socket.cleanupClients();
  }

  delay(timing::LOOP_MS);
}
