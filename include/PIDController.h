#pragma once

#include <Arduino.h>
#include <math.h>

#include "ParkingConfig.h"

// main's PID, with the names changed and nothing else. The error is a length in
// centimetres and the output is a servo angle offset in degrees. The gains, the
// integral clamp and the derivative low pass are main's and have not been tuned
// on this car.

class PIDController {
public:
  PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

  void reset();
  float compute(float errorCm, float dtSec);
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

// One servo offset from the lateral error and the loop period. A small error is
// treated as zero so the servo does not chase sensor noise, and the integral
// bleeds away inside that band. It is clamped so a long reverse cannot wind it
// up past the servo travel.
inline float PIDController::compute(float errorCm, float dtSec) {
  if (dtSec <= 0.001f) dtSec = 0.025f;
  if (fabsf(errorCm) < pid::DEADBAND_CM) errorCm = 0.0f;

  const float pTerm = kp_ * errorCm;

  if (errorCm != 0.0f) {
    integral_ += errorCm * dtSec;
    integral_ = constrain(integral_, -pid::INTEGRAL_CLAMP, pid::INTEGRAL_CLAMP);
  } else {
    integral_ *= 0.95f;
  }
  const float iTerm = ki_ * integral_;

  // The derivative is low passed because the ToF reading is noisy.
  const float rawDerivative = (errorCm - previousError_) / dtSec;
  filteredDerivative_ = pid::DERIVATIVE_ALPHA * rawDerivative +
                        (1.0f - pid::DERIVATIVE_ALPHA) * filteredDerivative_;
  const float dTerm = kd_ * filteredDerivative_;

  previousError_ = errorCm;
  return pTerm + iTerm + dTerm;
}

// Turn a servo offset into a whole degree inside the mechanical range. main
// rounded once and clamped, and that order is kept. With INVERTED false a
// positive offset commands an angle above centre.
inline int PIDController::toServoAngle(float offsetDeg) {
  if (servo::INVERTED) offsetDeg = -offsetDeg;
  const int angle = servo::CENTRE_DEG + (int)roundf(offsetDeg);
  return constrain(angle, servo::MIN_DEG, servo::MAX_DEG);
}
