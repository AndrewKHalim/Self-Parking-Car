#pragma once

// The hardware abstraction the control code talks to. The same controller
// compiles against the real ESP32 HAL and against the host simulator, so the
// analysis code can never drift from the firmware (PLAN.md section 14).

struct Hal {
  virtual ~Hal() = default;

  // Monotonic seconds. ---
  virtual float now() = 0;
  // end

  // Side and rear gaps in metres. A reading outside the valid range has to be
  // reported as such (see geometry.h validRange), not clamped. ---
  virtual void readSensors(float& gL, float& gR, float& gRear) = 0;
  // end

  // Commanded servo angle in degrees, where 90 is centre. ---
  virtual void setServo(float deg) = 0;
  // end

  // Drive command: 1 forward, -1 reverse, 0 brake (DRV8833 IN1=IN2=1). ---
  virtual void setMotor(int dir) = 0;
  // end
};
