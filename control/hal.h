#pragma once

// the hardware abstraction the control code talks to. the same controller
// compiles against the real ESP32 HAL and against the host simulator, so the
// analysis code cannot drift from the firmware (PLAN.md section 14).

struct Hal {
  virtual ~Hal() = default;

  // monotonic seconds. ---
  virtual float now() = 0;
  // end

  // side and rear gaps in metres. a reading outside the valid range has to be
  // reported as such (see geometry.h validRange), not clamped. ---
  virtual void readSensors(float& gL, float& gR, float& gRear) = 0;
  // end

  // commanded servo angle in degrees, where 90 is centre. ---
  virtual void setServo(float deg) = 0;
  // end

  // drive command: 1 forward, -1 reverse, 0 brake. the DRV8833 and the LM298
  // have the same truth table, so the firmware does not branch on which one is
  // fitted: one pin high and the other low drives, both high brakes fast, both
  // low brakes slowly. what differs is the voltage the driver drops and how
  // hard it brakes, and those live in the config, not the firmware. ---
  virtual void setMotor(int dir) = 0;
  // end
};
