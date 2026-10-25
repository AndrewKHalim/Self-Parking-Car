#include "DistanceSensorManager.h"
#include "MotorController.h"
#include "PIDController.h"
#include "ParkingConfig.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

// ============================================================================
// Global System Instances
// ============================================================================
DistanceSensorManager sensorManager;
PIDController pidController(1.8f, 0.04f, 0.40f); // Default Kp, Ki, Kd
MotorController motor;
Servo steeringServo;

// ============================================================================
// Parking System State Machine
// ============================================================================
enum ParkingState {
  STATE_BOOT = 0,
  STATE_IDLE_READY,
  STATE_REVERSE_PARKING,
  STATE_PULL_FORWARD_CORRECT,
  STATE_PARKED,
  STATE_EMERGENCY_STOP
};

ParkingState currentState = STATE_BOOT;
unsigned long stateStartTime = 0;
unsigned long lastLoopTime = 0;
int pullForwardCount = 0;
constexpr int MAX_PULL_FORWARD_ATTEMPTS = 3;

void setParkingState(ParkingState newState) {
  currentState = newState;
  stateStartTime = millis();
}

const char *getStateName(ParkingState state) {
  switch (state) {
  case STATE_BOOT:
    return "BOOT";
  case STATE_IDLE_READY:
    return "IDLE_READY";
  case STATE_REVERSE_PARKING:
    return "REVERSE_PARK";
  case STATE_PULL_FORWARD_CORRECT:
    return "PULL_FORWARD";
  case STATE_PARKED:
    return "PARKED_DONE";
  case STATE_EMERGENCY_STOP:
    return "EMERGENCY_STOP";
  default:
    return "UNKNOWN";
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(10);
  }
  delay(300);

  Serial.println("\n=======================================================");
  Serial.println("  Autonomous Reverse Parking System with Ackerman PID  ");
  Serial.println("=======================================================");

  // 1. Initialize I2C Bus for ESP32-S3
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); // 400 kHz fast I2C mode
  Serial.println("✓ I2C initialized on SDA=" + String(PIN_I2C_SDA) +
                 ", SCL=" + String(PIN_I2C_SCL));

  // 2. Attach and Center Steering Servo
  steeringServo.attach(PIN_SERVO_STEERING);
  steeringServo.write(SERVO_CENTER_DEG);
  Serial.println("✓ Steering servo centered at " + String(SERVO_CENTER_DEG) +
                 "° on pin " + String(PIN_SERVO_STEERING));

  // 3. Initialize Drive Motor
  motor.begin();

  // 4. Initialize Sensor Array (Sequential XSHUT initialization)
  Serial.println("Initializing 3x VL53L0X ToF sensor array...");
  if (!sensorManager.begin()) {
    Serial.println(
        "❌ CRITICAL: Sensor initialization failed. Halting system.");
    while (true) {
      delay(500);
    }
  }

  Serial.println("✓ All sensors online and calibrated.");
  Serial.println("Entering IDLE_READY. Starting parking in 2 seconds...\n");

  setParkingState(STATE_IDLE_READY);
  lastLoopTime = millis();
}

// ============================================================================
// Main Control Loop
// ============================================================================
void loop() {
  unsigned long now = millis();
  float deltaTimeSec = (now - lastLoopTime) / 1000.0f;
  if (deltaTimeSec <= 0.0f)
    deltaTimeSec = 0.01f;
  lastLoopTime = now;

  // 1. Read Distance Sensors (Filtered, Trigonometry-projected,
  // Proximity-guarded)
  SensorReadings readings = sensorManager.read();

  // 2. Compute Lateral Centering Error
  // Positive error  -> Closer to right obstacle -> Needs rear to steer left
  // Negative error  -> Closer to left obstacle  -> Needs rear to steer right
  float lateralError = 0.0f;
  bool bothSidesDetected = readings.leftValid && readings.rightValid;

  if (bothSidesDetected) {
    lateralError = readings.lateralLeftCm - readings.lateralRightCm;
  } else if (readings.leftValid && !readings.rightValid) {
    // Only left side detected: maintain ~12cm nominal lateral clearance
    constexpr float NOMINAL_CLEARANCE_CM = 12.0f;
    lateralError = (NOMINAL_CLEARANCE_CM - readings.lateralLeftCm);
  } else if (!readings.leftValid && readings.rightValid) {
    // Only right side detected: maintain ~12cm nominal lateral clearance
    constexpr float NOMINAL_CLEARANCE_CM = 12.0f;
    lateralError = (readings.lateralRightCm - NOMINAL_CLEARANCE_CM);
  } else {
    // Open area: maintain straight trajectory
    lateralError = 0.0f;
  }

  // 3. Finite State Machine
  int targetServoAngle = SERVO_CENTER_DEG;
  float leftWheelDeg = 0.0f;
  float rightWheelDeg = 0.0f;

  switch (currentState) {
  case STATE_IDLE_READY: {
    motor.stop();
    steeringServo.write(SERVO_CENTER_DEG);
    if (now - stateStartTime > 2000) {
      Serial.println("🚀 [STATE] Beginning Autonomous Reverse Parking!");
      pidController.reset();
      setParkingState(STATE_REVERSE_PARKING);
    }
    break;
  }

  case STATE_REVERSE_PARKING: {
    // Safety Check: Proximity hazard (< 3.5 cm) on any sensor
    if (readings.proximityAlert ||
        readings.filteredBackCm < SENSOR_EMERGENCY_CM) {
      Serial.println("⚠️ [PROXIMITY ALERT] Obstacle dangerously close! "
                     "Triggering emergency stop.");
      motor.stop();
      setParkingState(STATE_EMERGENCY_STOP);
      break;
    }

    // PID Steering Computation
    float pidOutput = pidController.compute(lateralError, deltaTimeSec);
    targetServoAngle =
        PIDController::mapToServoAngle(pidOutput, STEERING_INVERTED);
    steeringServo.write(targetServoAngle);

    // Compute actual Ackerman inner/outer road wheel angles
    PIDController::getAckermanWheelAngles(targetServoAngle, leftWheelDeg,
                                          rightWheelDeg);

    // Speed Control: Speed modulation based on rear distance
    if (readings.filteredBackCm <= APPROACH_SLOW_DIST_CM) {
      // Precision crawl speed as we near the rear wall
      motor.reverse(SPEED_REVERSE_CRAWL);
    } else {
      // Standard reverse cruising speed
      motor.reverse(SPEED_REVERSE_CRUISE);
    }

    // --- Termination & Transition Conditions ---

    // Condition A: Successfully reached rear target distance (~7.0 cm)
    if (readings.filteredBackCm <= (TARGET_BACK_WALL_CM + BACK_WALL_TOLERANCE_CM) ||
        (readings.backValid && readings.rawBackCm <= (TARGET_BACK_WALL_CM + BACK_WALL_TOLERANCE_CM))) {
      motor.stop();
      steeringServo.write(SERVO_CENTER_DEG);
      setParkingState(STATE_PARKED);
      Serial.printf("🎉 [SUCCESS] Car parked accurately! Final rear distance: %.1f cm (raw: %.1f cm)\n",
                    readings.filteredBackCm, readings.rawBackCm);
      break;
    }

    // Condition B: Approaching rear wall but lateral alignment is poor -> Pull
    // forward to adjust
    if (readings.filteredBackCm < PULL_FORWARD_THRESH_CM &&
        fabs(lateralError) > ALIGNMENT_TOLERANCE_CM &&
        pullForwardCount < MAX_PULL_FORWARD_ATTEMPTS) {

      Serial.printf("🔄 [ALIGNMENT] Close to back wall (%.1f cm) with error "
                    "%.1f cm. Pulling forward to adjust (Attempt %d/%d)...\n",
                    readings.filteredBackCm, lateralError, pullForwardCount + 1,
                    MAX_PULL_FORWARD_ATTEMPTS);
      motor.stop();
      pullForwardCount++;
      setParkingState(STATE_PULL_FORWARD_CORRECT);
      break;
    }
    break;
  }

  case STATE_PULL_FORWARD_CORRECT: {
    // Steer gently opposite to error to square up the car while moving forward
    float counterSteer = (lateralError > 0) ? -25.0f : 25.0f;
    targetServoAngle =
        PIDController::mapToServoAngle(counterSteer, STEERING_INVERTED);
    steeringServo.write(targetServoAngle);
    PIDController::getAckermanWheelAngles(targetServoAngle, leftWheelDeg,
                                          rightWheelDeg);

    motor.forward(SPEED_FORWARD_ADJUST);

    // Pull forward for preset duration or until rear clearance reaches safe
    // distance (> 22 cm)
    if (now - stateStartTime >= PULL_FWD_DURATION_MS ||
        readings.filteredBackCm > 22.0f) {
      motor.stop();
      steeringServo.write(SERVO_CENTER_DEG);
      pidController.reset();
      delay(150); // Pause briefly before switching gear
      setParkingState(STATE_REVERSE_PARKING);
      Serial.println("↩ Resuming reverse parking with centered alignment.");
    }
    break;
  }

  case STATE_PARKED: {
    motor.stop();
    steeringServo.write(SERVO_CENTER_DEG);
    targetServoAngle = SERVO_CENTER_DEG;
    leftWheelDeg = 0.0f;
    rightWheelDeg = 0.0f;
    break;
  }

  case STATE_EMERGENCY_STOP: {
    motor.stop();
    steeringServo.write(SERVO_CENTER_DEG);
    // If obstacle moves away or cleared, allow recovery
    if (!readings.proximityAlert && readings.filteredBackCm > 8.0f) {
      Serial.println("Obstacle cleared. Resuming reverse parking.");
      setParkingState(STATE_REVERSE_PARKING);
    }
    break;
  }

  default:
    break;
  }

  // 4. Telemetry Output (Structured serial logging at ~10 Hz)
  static unsigned long lastLogTime = 0;
  if (now - lastLogTime >= 100) {
    lastLogTime = now;
    Serial.printf(
        "[%s] Back: %5.1f cm | LatL: %5.1f cm | LatR: %5.1f cm | Err: %+5.1f "
        "cm | Servo: %3d° | Whl(L/R): %+4.1f°/%+4.1f° | Mot: %s(%d)\n",
        getStateName(currentState), readings.filteredBackCm,
        readings.lateralLeftCm, readings.lateralRightCm, lateralError,
        targetServoAngle, leftWheelDeg, rightWheelDeg,
        motor.getDirectionString(), motor.getSpeed());
  }

  delay(25); // Loop cadence (~30-40 Hz)
}