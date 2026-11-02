// Lateral PID. Evaluated once per parking cycle, only while the car is braked
// and settled, from the left/right gap difference. The rear distance is never
// an input: it only says when to stop reversing.
#pragma once

#include "config.h"

class LateralPID {
public:
    void reset() {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
        has_prev_ = false;
    }

    // dt_s is the time since the previous evaluation. The controller supplies
    // 1.0 s for the first sample of a cycle, when there is no previous
    // timestamp. The derivative stays at 0 there because there is no previous
    // error to difference against.
    float computeWheelDeg(float error_mm, float dt_s, const TuningConfig& cfg) {
        if (cfg.ki > 0.0f && dt_s > 0.001f) {
            integral_ += error_mm * dt_s;
            // Keep the integral contribution equal to the output clamp; the
            // term is measured in wheel degrees, not in error-mm-seconds.
            const float limit = pid_defaults::MAX_I_WHEEL_DEG / cfg.ki;
            integral_ = util::clampf(integral_, -limit, limit);
        }
        if (cfg.ki <= 0.0f) {
            integral_ = 0.0f;
        }

        const float d =
            (has_prev_ && dt_s > 0.001f) ? (error_mm - prev_error_) / dt_s : 0.0f;
        prev_error_ = error_mm;
        has_prev_ = true;

        const float out = cfg.kp * error_mm + cfg.ki * integral_ + cfg.kd * d;
        return util::clampf(out, -steering::MAX_WHEEL_DEG, steering::MAX_WHEEL_DEG);
    }

private:
    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
    bool has_prev_ = false;
};
