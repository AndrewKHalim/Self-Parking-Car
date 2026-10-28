#pragma once

#include <cstdint>

// The hardware the control code talks to. The same controller builds against the ESP32 HAL and the host simulator, so analysis cannot drift from firmware.

struct Hal {
  virtual ~Hal() = default;

  // Monotonic seconds.
  virtual float now() = 0;

  // Gaps in metres, reported invalid rather than clamped.
  virtual void readSensors(float& gL, float& gR, float& gRear) = 0;

  // Servo angle in degrees, 90 is centre.
  virtual void setServo(float deg) = 0;

  // 1 forward, -1 reverse, 0 brake. Duty is 0 to 255 and ignored when dir is 0.
  // Both drivers share a truth table, so the firmware does not branch on which
  // is fitted, and the simulator ignores the duty because its plant takes the
  // speed from Config::speed.
  virtual void setMotor(int dir, uint8_t duty = 255) = 0;
};
