// Three-step parking maneuver.
//
//   1. reverse straight (servo centred) until the rear sensor reads the target
//   2. stop, settle, and if the gaps differ, compute a steer angle from a
//      stationary PID sample and crawl forward for a fixed time
//   3. reverse straight back to the target and repeat step 2 until the gaps
//      agree within the tolerance
//
// Nothing is computed while the car is moving. The only inputs are distances;
// the rear sensor decides when to stop, the side sensors decide where to aim.
#pragma once

#include <math.h>
#include <stdint.h>

#include "actuators.h"
#include "config.h"
#include "pid.h"
#include "sensors.h"

enum class Phase : uint8_t {
    IDLE = 0,
    REVERSE,
    SETTLE_REAR,
    STEER_WAIT,
    FORWARD_CORRECT,
    SETTLE_FORWARD,
    PARKED,
    FAULT,
};

inline const char* phaseName(Phase p) {
    switch (p) {
        case Phase::IDLE: return "idle";
        case Phase::REVERSE: return "reverse";
        case Phase::SETTLE_REAR: return "settle_rear";
        case Phase::STEER_WAIT: return "steer";
        case Phase::FORWARD_CORRECT: return "forward";
        case Phase::SETTLE_FORWARD: return "settle_fwd";
        case Phase::PARKED: return "parked";
        case Phase::FAULT: return "fault";
    }
    return "unknown";
}

struct Telemetry {
    uint32_t t_ms;
    uint16_t gL_mm;
    uint16_t gR_mm;
    int16_t diff_mm;
    uint16_t rear_mm;
    uint16_t rear_target_mm;
    uint16_t align_diff_mm;
    uint8_t backoff_max;
    uint8_t duty;
    uint16_t steer_settle_ms;
    int16_t servo_deg;
    float wheel_deg;
    uint16_t cycles;
    uint8_t backoffs;
    Phase phase;
    const char* fault;
};

class ParkingMachine {
public:
    const TuningConfig& config() const { return cfg_; }
    Phase phase() const { return phase_; }

    void setRearTargetMm(uint16_t mm) {
        cfg_.rear_target_mm =
            (uint16_t)util::clampi(mm, defaults::REAR_TARGET_MIN_MM, defaults::REAR_TARGET_MAX_MM);
    }

    void setSteerSettleMs(uint16_t ms) {
        cfg_.steer_settle_ms = ms > defaults::STEER_SETTLE_MAX_MS ? defaults::STEER_SETTLE_MAX_MS : ms;
    }

    void setDuty(uint8_t duty) {
        cfg_.duty = duty > drive::DUTY_MAX ? drive::DUTY_MAX : duty;
    }

    void setAlignDiffMm(uint16_t mm) {
        cfg_.align_diff_mm =
            (uint16_t)util::clampi(mm, defaults::ALIGN_DIFF_MIN_MM, defaults::ALIGN_DIFF_MAX_MM);
    }

    // Zero means no cap: the car keeps pulling forward until Stop.
    void setBackoffMax(uint8_t count) {
        cfg_.backoff_max = count > defaults::BACKOFF_MAX_MAX ? defaults::BACKOFF_MAX_MAX : count;
    }

    // A negative gain means "leave this one alone", so the page can send only
    // the fields the operator changed.
    void setGains(float kp, float ki, float kd) {
        if (kp >= 0.0f) {
            cfg_.kp = util::clampf(kp, 0.0f, pid_defaults::KP_MAX);
        }
        if (ki >= 0.0f) {
            cfg_.ki = util::clampf(ki, 0.0f, pid_defaults::KI_MAX);
        }
        if (kd >= 0.0f) {
            cfg_.kd = util::clampf(kd, 0.0f, pid_defaults::KD_MAX);
        }
    }

    void start(uint32_t now_ms, const SensorReading& s, Actuators& act) {
        pid_.reset();
        cycles_ = 0;
        backoffs_ = 0;
        last_pid_ms_ = 0;
        wheel_deg_ = 0.0f;
        fault_ = "none";
        act.setServoDeg(steering::SERVO_CENTRE_DEG);

        if (!s.rear_valid) {
            act.brake();
            phase_ = Phase::FAULT;
            fault_ = "rear_sensor";
            return;
        }
        phase_start_ms_ = now_ms;
        if (s.rear_mm <= cfg_.rear_target_mm) {
            act.brake();
            phase_ = Phase::SETTLE_REAR;
            return;
        }
        act.reverse(cfg_.duty);
        phase_ = Phase::REVERSE;
    }

    void stop(Actuators& act) {
        act.brake();
        act.setServoDeg(steering::SERVO_CENTRE_DEG);
        pid_.reset();
        wheel_deg_ = 0.0f;
        phase_ = Phase::IDLE;
        fault_ = "none";
    }

    Telemetry tick(uint32_t now_ms, const SensorReading& s, Actuators& act) {
        const uint32_t elapsed = now_ms - phase_start_ms_;

        switch (phase_) {
            case Phase::IDLE:
            case Phase::PARKED:
            case Phase::FAULT:
                act.brake();
                break;

            case Phase::REVERSE:
                act.setServoDeg(steering::SERVO_CENTRE_DEG);
                act.reverse(cfg_.duty);
                if (s.rear_valid && atRearTarget(s.rear_mm)) {
                    act.brake();
                    phase_start_ms_ = now_ms;
                    phase_ = Phase::SETTLE_REAR;
                } else if (elapsed >= defaults::REVERSE_TIMEOUT_MS) {
                    act.brake();
                    phase_ = Phase::FAULT;
                    fault_ = "rev_timeout";
                }
                break;

            case Phase::SETTLE_REAR:
                act.brake();
                act.setServoDeg(steering::SERVO_CENTRE_DEG);
                if (elapsed >= cfg_.settle_ms) {
                    settleRear(now_ms, s, act);
                }
                break;

            case Phase::STEER_WAIT:
                // Wheels first, motors second: hold the angle and stay braked
                // until the servo has had time to arrive.
                act.brake();
                act.setWheelAngleDeg(wheel_deg_);
                if (elapsed >= turnWaitMs()) {
                    act.forward(cfg_.duty);
                    phase_start_ms_ = now_ms;
                    phase_ = Phase::FORWARD_CORRECT;
                }
                break;

            case Phase::FORWARD_CORRECT:
                act.forward(cfg_.duty);
                if (elapsed >= move_ms_) {
                    act.brake();
                    act.setServoDeg(steering::SERVO_CENTRE_DEG);
                    wheel_deg_ = 0.0f;
                    phase_start_ms_ = now_ms;
                    phase_ = Phase::SETTLE_FORWARD;
                }
                break;

            case Phase::SETTLE_FORWARD:
                act.brake();
                act.setServoDeg(steering::SERVO_CENTRE_DEG);
                if (elapsed >= turnWaitMs()) {
                    phase_start_ms_ = now_ms;
                    if (s.rear_valid && atRearTarget(s.rear_mm)) {
                        phase_ = Phase::SETTLE_REAR;
                    } else {
                        act.reverse(cfg_.duty);
                        phase_ = Phase::REVERSE;
                    }
                }
                break;
        }

        return snapshot(now_ms, s, act);
    }

private:
    // Whichever wait is longer: the chassis settling and the servo travelling
    // are different clocks.
    uint32_t turnWaitMs() const {
        return cfg_.steer_settle_ms > cfg_.settle_ms ? cfg_.steer_settle_ms : cfg_.settle_ms;
    }

    // The band the tail is allowed: half the tolerance above the target. Braking
    // at the top of it leaves room for the frame the brake costs.
    bool atRearTarget(uint16_t rear_mm) const {
        return rear_mm <= (uint16_t)(cfg_.rear_target_mm + defaults::REAR_TOLERANCE_MM);
    }

    // Stationary sample: either we are aligned and done, or we turn the wheels
    // by the PID result and move forward.
    void settleRear(uint32_t now_ms, const SensorReading& s, Actuators& act) {
        if (!s.left_valid || !s.right_valid) {
            act.brake();
            phase_ = Phase::FAULT;
            fault_ = "side_sensor";
            return;
        }

        const int16_t diff = (int16_t)s.gL_mm - (int16_t)s.gR_mm;
        const int abs_diff = diff < 0 ? -diff : diff;
        if (abs_diff <= (int)cfg_.align_diff_mm) {
            // Aligned, but if the tail stopped closer than the target band the
            // run is not over: pull straight forward and come back at it. This
            // is what recovers from a late brake.
            const int rear_too_close =
                (int)s.rear_mm + (int)defaults::REAR_TOLERANCE_MM < (int)cfg_.rear_target_mm;
            if (s.rear_valid && rear_too_close) {
                // A capped run stops here rather than pulling at the wall for
                // ever: the gaps are aligned, the distance is what failed.
                if (cfg_.backoff_max != 0 && backoffs_ >= cfg_.backoff_max) {
                    act.brake();
                    phase_ = Phase::FAULT;
                    fault_ = "backoff_limit";
                    return;
                }
                ++backoffs_;
                act.setServoDeg(steering::SERVO_CENTRE_DEG);
                act.forward(cfg_.duty);
                move_ms_ = defaults::BACKOFF_MOVE_MS;
                phase_start_ms_ = now_ms;
                phase_ = Phase::FORWARD_CORRECT;
                return;
            }
            act.brake();
            act.setServoDeg(steering::SERVO_CENTRE_DEG);
            phase_ = Phase::PARKED;
            return;
        }

        // First sample of a cycle has no previous timestamp; the plan fixes that
        // interval at one second so the integral term still moves.
        const float dt_s =
            (last_pid_ms_ == 0) ? 1.0f : (float)(now_ms - last_pid_ms_) * 0.001f;
        last_pid_ms_ = now_ms;

        wheel_deg_ = pid_.computeWheelDeg((float)diff, dt_s, cfg_);
        act.setWheelAngleDeg(wheel_deg_);
        ++cycles_;

        // The motors stay off until the wheels are where the PID asked.
        move_ms_ = cfg_.forward_move_ms;
        phase_start_ms_ = now_ms;
        phase_ = Phase::STEER_WAIT;
    }

    Telemetry snapshot(uint32_t now_ms, const SensorReading& s, Actuators& act) const {
        Telemetry t;
        t.t_ms = now_ms;
        t.gL_mm = s.gL_mm;
        t.gR_mm = s.gR_mm;
        t.diff_mm = (int16_t)s.gL_mm - (int16_t)s.gR_mm;
        t.rear_mm = s.rear_mm;
        t.rear_target_mm = cfg_.rear_target_mm;
        t.align_diff_mm = cfg_.align_diff_mm;
        t.backoff_max = cfg_.backoff_max;
        t.duty = cfg_.duty;
        t.steer_settle_ms = (uint16_t)cfg_.steer_settle_ms;
        t.servo_deg = (int16_t)act.servoDeg();
        t.wheel_deg = wheel_deg_;
        t.cycles = cycles_;
        t.backoffs = (uint8_t)backoffs_;
        t.phase = phase_;
        t.fault = fault_;
        return t;
    }

    TuningConfig cfg_;
    LateralPID pid_;
    Phase phase_ = Phase::IDLE;
    uint32_t phase_start_ms_ = 0;
    uint32_t last_pid_ms_ = 0;
    uint32_t move_ms_ = defaults::FORWARD_MOVE_MS;  // current forward leg
    uint16_t backoffs_ = 0;
    float wheel_deg_ = 0.0f;
    uint16_t cycles_ = 0;
    const char* fault_ = "none";
};
