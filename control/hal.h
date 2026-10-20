#pragma once

// The hardware abstraction the control code talks to. The same controller
// compiles against the real ESP32 HAL and against the host simulator, so the
// analysis code can never drift from the firmware (PLAN.md section 14).

struct Hal {
  virtual ~Hal() = default;

  virtual float now() = 0;  // monotonic seconds

  // Side and rear gaps in metres. A reading outside the valid range must be
  // reported as such (see geometry.h validRange), not clamped.
  virtual void readSensors(float& gL, float& gR, float& gRear) = 0;

  // Commanded servo angle in degrees, 90 = centre.
  virtual void setServo(float deg) = 0;

  // Drive command: +1 forward, -1 reverse, 0 brake (DRV8833 IN1=IN2=1).
  virtual void setMotor(int dir) = 0;
};
