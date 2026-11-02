#pragma once

#include <Arduino.h>
#include <math.h>

#include "ParkingConfig.h"

// One PID on the lateral error. The error is a length in metres and the output
// is a servo angle offset in degrees. The gains, the integral clamp and the
// derivative low pass are inherited from main and are unverified on this car.

class PIDController {
public:
  PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

  void reset();
  float compute(float error, float dt);
  static int toServoAngle(float offsetDeg);

private:
  float kp_;
  float ki_;
  float kd_;
  float integral_ = 0.0f;
  float previousError_ = 0.0f;
  float filteredDerivative_ = 0.0f;
};

inline void PIDController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  filteredDerivative_ = 0.0f;
}

// Compute one servo offset from the lateral error and the loop period.
inline float PIDController::compute(float error, float dt) {
  if (dt <= 0.001f) dt = 0.025f;

  // A small error is treated as zero so the servo does not chase sensor noise.
  if (fabsf(error) < pid::DEADBAND_M) error = 0.0f;
  // end

  const float pTerm = kp_ * error;

  // The integral is clamped and bleeds away inside the deadband, which keeps a
  // long reverse from winding it up.
  if (error != 0.0f) {
    integral_ += error * dt;
    integral_ = constrain(integral_, -pid::INTEGRAL_CLAMP,
                          pid::INTEGRAL_CLAMP);
  } else {
    integral_ *= 0.95f;
  }
  const float iTerm = ki_ * integral_;
  // end

  // The derivative is low passed because the ToF reading is noisy.
  const float rawDerivative = (error - previousError_) / dt;
  filteredDerivative_ = pid::DERIVATIVE_ALPHA * rawDerivative +
                        (1.0f - pid::DERIVATIVE_ALPHA) * filteredDerivative_;
  const float dTerm = kd_ * filteredDerivative_;
  // end

  previousError_ = error;
  return pTerm + iTerm + dTerm;
}

// Turn a servo offset into a whole degree inside the mechanical range. DIR
// carries the steering sign so the owner can flip it in one place.
inline int PIDController::toServoAngle(float offsetDeg) {
  const int angle = servo::CENTRE_DEG + servo::DIR * (int)lroundf(offsetDeg);
  return constrain(angle, servo::MIN_DEG, servo::MAX_DEG);
}
