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
  const float previousIntegral = integral_;

  // The integral is clamped and bleeds away inside the deadband. The clamp
  // alone can still wind up against a servo stop, so the rail check below
  // rolls the update back.
  if (error != 0.0f) {
    integral_ += error * dt;
    integral_ = constrain(integral_, -pid::INTEGRAL_CLAMP,
                          pid::INTEGRAL_CLAMP);
  } else {
    integral_ *= 0.95f;
  }
  float iTerm = ki_ * integral_;
  // end

  // The derivative is low passed because the ToF reading is noisy.
  const float rawDerivative = (error - previousError_) / dt;
  filteredDerivative_ = pid::DERIVATIVE_ALPHA * rawDerivative +
                        (1.0f - pid::DERIVATIVE_ALPHA) * filteredDerivative_;
  const float dTerm = kd_ * filteredDerivative_;
  // end

  float output = pTerm + iTerm + dTerm;

  // Freeze the integral while the command is against a stop and the error still
  // pushes that way. Rolling back the update is what stops a long reverse from
  // winding the integral up behind a railed servo command. An error that pulls
  // away from the stop is still allowed to unwind it.
  const int angle = toServoAngle(output);
  const bool pushingHigh =
      angle >= servo::MAX_DEG && servo::DIR * error > 0.0f;
  const bool pushingLow =
      angle <= servo::MIN_DEG && servo::DIR * error < 0.0f;
  if (pushingHigh || pushingLow) {
    integral_ = previousIntegral;
    iTerm = ki_ * integral_;
    output = pTerm + iTerm + dTerm;
  }
  // end

  previousError_ = error;
  return output;
}

// Turn a servo offset into a whole degree inside the mechanical range. The
// rounding and the DIR multiply each happen once. With DIR at 1 a positive
// offset commands an angle above centre, which is the direction that should
// turn the rear toward the car's left. Flip DIR if the linkage turns the other
// way.
inline int PIDController::toServoAngle(float offsetDeg) {
  const int angle = servo::CENTRE_DEG + servo::DIR * (int)lroundf(offsetDeg);
  return constrain(angle, servo::MIN_DEG, servo::MAX_DEG);
}
