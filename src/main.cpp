#include "DistanceSensorManager.h"
#include "MotorController.h"
#include "PIDController.h"
#include "ParkingConfig.h"

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>

// The whole firmware. One loop, one PID on the lateral error, one small state
// machine and one serial line. The loop runs at 40 Hz and the telemetry prints
// at 10 Hz. Nothing here owns a task, a socket or a web page.

DistanceSensorManager sensorManager;
PIDController pidController(pid::KP, pid::KI, pid::KD);
MotorController motor;
Servo steeringServo;

enum ParkingState {
  STATE_IDLE = 0,
  STATE_REVERSE,
  STATE_PULL_FORWARD,
  STATE_PARKED,
  STATE_EMERGENCY_STOP
};

ParkingState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long lastLoopTime = 0;
int pullForwardCount = 0;

void setParkingState(ParkingState newState) {
  currentState = newState;
  stateStartTime = millis();
}

const char* getStateName(ParkingState state) {
  switch (state) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_REVERSE:
    return "REVERSE";
  case STATE_PULL_FORWARD:
    return "PULL_FORWARD";
  case STATE_PARKED:
    return "PARKED";
  case STATE_EMERGENCY_STOP:
    return "EMERGENCY_STOP";
  default:
    return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  const unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) {
    delay(10);
  }
  delay(300);

  Serial.println();
  Serial.println("autoparking-car: simple branch");
  Serial.println("one loop, one PID, serial telemetry");

  Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);
  Wire.setClock(400000);

  // Attach with the library defaults and write whole degrees, which is the
  // pulse map main drove this servo with.
  steeringServo.attach(board::PIN_SERVO);
  steeringServo.write(servo::CENTRE_DEG);
  // end

  motor.begin();

  if (!sensorManager.begin()) {
    Serial.println("sensor bring-up failed, halted");
    while (true) {
      delay(500);
    }
  }

  Serial.println("sensors online, starting in 2 s");
  setParkingState(STATE_IDLE);
  lastLoopTime = millis();
}

void loop() {
  const unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0f;
  if (dt <= 0.001f) dt = 0.025f;
  lastLoopTime = now;

  const SensorReadings readings = sensorManager.read();

  // The side faces are perpendicular to the car, so the gaps need no trig.
  // Positive error means the left gap is larger, so the car sits to the right
  // of the bay centre and the rear has to be steered toward the left wall.
  const bool bothSides = readings.leftValid && readings.rightValid;
  const float lateralError = bothSides ? readings.gL - readings.gR : 0.0f;
  // end

  // Any valid reading under 40 mm is an emergency. It latches, because a
  // transient close reading should not silently restart a moving car.
  const bool tooClose =
      (readings.leftValid && readings.gL < parking::EMERGENCY_STOP_M) ||
      (readings.rightValid && readings.gR < parking::EMERGENCY_STOP_M) ||
      (readings.rearValid && readings.gRear < parking::EMERGENCY_STOP_M);
  if (tooClose && currentState != STATE_PARKED &&
      currentState != STATE_EMERGENCY_STOP) {
    setParkingState(STATE_EMERGENCY_STOP);
  }
  // end

  int servoAngle = servo::CENTRE_DEG;

  switch (currentState) {
  case STATE_IDLE:
    motor.stop();
    steeringServo.write(servo::CENTRE_DEG);
    if (now - stateStartTime >= parking::IDLE_MS) {
      pidController.reset();
      setParkingState(STATE_REVERSE);
    }
    break;

  case STATE_REVERSE: {
    const float pidOutput = pidController.compute(lateralError, dt);
    servoAngle = PIDController::toServoAngle(pidOutput);
    steeringServo.write(servoAngle);

    // Reaching the rear trigger stops the car and ends the maneuver. The
    // reading is the tail clearance, so the 50 mm the car actually keeps comes
    // from the 20 mm it rolls on after the drive is cut.
    if (readings.rearValid &&
        readings.gRear <= parking::REVERSE_TRIGGER_M) {
      motor.stop();
      steeringServo.write(servo::CENTRE_DEG);
      setParkingState(STATE_PARKED);
      break;
    }
    // end

    // Inside the last stretch, a lateral error worth correcting is worth a
    // pull forward. The counter is capped so a wall that cannot be reached
    // does not keep the car hunting.
    if (readings.rearValid && readings.gRear <= parking::CRAWL_BELOW_M &&
        fabsf(lateralError) > parking::ALIGN_TOL_M &&
        pullForwardCount < parking::MAX_PULL_FORWARD) {
      motor.stop();
      ++pullForwardCount;
      setParkingState(STATE_PULL_FORWARD);
      break;
    }
    // end

    if (readings.rearValid && readings.gRear <= parking::CRAWL_BELOW_M) {
      motor.reverse(drive::DUTY_CRAWL);
    } else {
      motor.reverse(drive::DUTY_REVERSE);
    }
    break;
  }

  case STATE_PULL_FORWARD: {
    const float counterSteer = (lateralError > 0.0f)
                                   ? -parking::PULL_FORWARD_STEER_DEG
                                   : parking::PULL_FORWARD_STEER_DEG;
    servoAngle = PIDController::toServoAngle(counterSteer);
    steeringServo.write(servoAngle);
    motor.forward(drive::DUTY_FORWARD);

    // The stroke ends on time or as soon as the rear has room again.
    const bool timedOut = now - stateStartTime >= parking::PULL_FORWARD_MS;
    const bool cleared =
        readings.rearValid && readings.gRear > parking::PULL_FORWARD_EXIT_M;
    if (timedOut || cleared) {
      motor.stop();
      steeringServo.write(servo::CENTRE_DEG);
      pidController.reset();
      setParkingState(STATE_REVERSE);
    }
    // end
    break;
  }

  case STATE_PARKED:
    motor.stop();
    steeringServo.write(servo::CENTRE_DEG);
    servoAngle = servo::CENTRE_DEG;
    break;

  case STATE_EMERGENCY_STOP:
    motor.stop();
    steeringServo.write(servo::CENTRE_DEG);
    servoAngle = servo::CENTRE_DEG;
    break;

  default:
    break;
  }

  // One line at 10 Hz, main's field order in metres and whole servo degrees.
  static unsigned long lastLogTime = 0;
  if (now - lastLogTime >= 100) {
    lastLogTime = now;
    Serial.printf("[%s] Rear: %.3f m | Left: %.3f m | Right: %.3f m | "
                  "Err: %+.3f m | Servo: %3d deg | Mot: %s(%u)\n",
                  getStateName(currentState), readings.gRear, readings.gL,
                  readings.gR, lateralError, servoAngle,
                  motor.getDirectionString(), motor.getSpeed());
  }

  delay(25);
}
