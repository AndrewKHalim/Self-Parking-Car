// Parking maneuver, now with a heading gate.
//
//   1. reverse straight (servo centred) until the rear sensor reads the target
//   2. stop, settle, and estimate the car's yaw from the change in (gL - gR)
//      over the reverse leg just driven. Because the side sensors are 80 mm
//      apart longitudinally, that difference also carries a heading term; the
//      stagger coefficient removes it and leaves the true lateral offset.
//   3. if the offset is wrong, crawl forward along an S-curve: the first half
//      steers toward the gap, the second half counter-steers by the same amount
//      so the lateral shift happens without adding heading. If only the heading
//      is wrong, one short straightening stroke does the same job.
//   4. reverse straight back to the target and repeat until both the offset and
//      the yaw are inside their tolerances.
//
// Nothing is computed while the car is moving, except the reverse-leg anchor
// that the slope estimator needs. The rear sensor decides when to stop, the
// side pair decides where to aim and how square the car is.
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
    STEER_COUNTER_WAIT,
    FORWARD_COUNTER,
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
        case Phase::STEER_COUNTER_WAIT: return "steer_counter";
        case Phase::FORWARD_COUNTER: return "forward_counter";
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
    int16_t diff_mm;      // stagger-compensated offset: (gL - gR) - 51 * tan(psi)
    int16_t raw_diff_mm;  // what the two sensors actually returned
    uint16_t rear_mm;
    uint16_t rear_target_mm;
    uint16_t align_diff_mm;
    uint8_t backoff_max;
    uint8_t duty;
    uint16_t steer_settle_ms;
    int16_t servo_deg;
    float wheel_deg;
    float yaw_deg;
    float yaw_tol_deg;
    float k_yaw;
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

    void setYawTolDeg(float deg) {
        cfg_.yaw_tol_deg =
            util::clampf(deg, defaults::YAW_TOL_MIN_DEG, defaults::YAW_TOL_MAX_DEG);
    }

    void setKYaw(float k) {
        cfg_.k_yaw = util::clampf(k, 0.0f, pid_defaults::K_YAW_MAX);
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
        second_leg_ms_ = 0;
        counter_wheel_deg_ = 0.0f;
        rev_start_valid_ = false;
        tan_psi_ = 0.0f;
        yaw_deg_ = 0.0f;
        yaw_valid_ = false;
        corr_diff_mm_ = (int16_t)s.gL_mm - (int16_t)s.gR_mm;
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
        recordReverseStart(s);
        act.reverse(cfg_.duty);
        phase_ = Phase::REVERSE;
    }

    void stop(Actuators& act) {
        act.brake();
        act.setServoDeg(steering::SERVO_CENTRE_DEG);
        pid_.reset();
        wheel_deg_ = 0.0f;
        second_leg_ms_ = 0;
        counter_wheel_deg_ = 0.0f;
        rev_start_valid_ = false;
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
                // Latch the anchor on the first tick where all three sensors
                // are usable: start() may have run before the first read.
                if (!rev_start_valid_) {
                    recordReverseStart(s);
                }
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
                    if (second_leg_ms_ > 0) {
                        // Second half of the S-curve: opposite lock, same time.
                        wheel_deg_ = counter_wheel_deg_;
                        act.setWheelAngleDeg(wheel_deg_);
                        phase_start_ms_ = now_ms;
                        phase_ = Phase::STEER_COUNTER_WAIT;
                    } else {
                        act.setServoDeg(steering::SERVO_CENTRE_DEG);
                        wheel_deg_ = 0.0f;
                        phase_start_ms_ = now_ms;
                        phase_ = Phase::SETTLE_FORWARD;
                    }
                }
                break;

            case Phase::STEER_COUNTER_WAIT:
                act.brake();
                act.setWheelAngleDeg(wheel_deg_);
                if (elapsed >= turnWaitMs()) {
                    act.forward(cfg_.duty);
                    phase_start_ms_ = now_ms;
                    phase_ = Phase::FORWARD_COUNTER;
                }
                break;

            case Phase::FORWARD_COUNTER:
                act.forward(cfg_.duty);
                if (elapsed >= second_leg_ms_) {
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
                        recordReverseStart(s);
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

    // Both side readings and the rear distance at the far end of a reverse leg.
    // The yaw estimate is the change between this anchor and the settled sample
    // at the stop, so a stale anchor is worse than none.
    void recordReverseStart(const SensorReading& s) {
        if (s.rear_valid && s.left_valid && s.right_valid) {
            rev_start_rear_mm_ = s.rear_mm;
            rev_start_diff_mm_ = (int16_t)s.gL_mm - (int16_t)s.gR_mm;
            rev_start_valid_ = true;
        } else {
            rev_start_valid_ = false;
        }
    }

    // Stationary sample: measure yaw and offset, then either declare the run
    // finished, pull straight back from a wall the car overshot, or steer the
    // S-curve that fixes what is still wrong.
    void settleRear(uint32_t now_ms, const SensorReading& s, Actuators& act) {
        if (!s.left_valid || !s.right_valid) {
            act.brake();
            phase_ = Phase::FAULT;
            fault_ = "side_sensor";
            return;
        }

        const int16_t raw_diff = (int16_t)s.gL_mm - (int16_t)s.gR_mm;

        // Reversing straight is a pure translation, so the only way (gL - gR)
        // can change over that leg is a heading error. Nose-right (psi > 0)
        // sends the tail rear-left: the car drifts toward the left wall and the
        // difference shrinks, hence diff_start - diff_stop in the slope.
        if (rev_start_valid_ && s.rear_valid && rev_start_rear_mm_ > s.rear_mm) {
            const int32_t d_rear = (int32_t)rev_start_rear_mm_ - (int32_t)s.rear_mm;
            if (d_rear >= (int32_t)defaults::MIN_REVERSE_SPAN_MM) {
                const float slope =
                    (float)(rev_start_diff_mm_ - raw_diff) / (2.0f * (float)d_rear);
                tan_psi_ = util::clampf(slope, -defaults::MAX_TAN_PSI, defaults::MAX_TAN_PSI);
                yaw_deg_ = atanf(tan_psi_) * 57.2957795f;
                yaw_valid_ = true;
            }
        }

        // The forward sensor sees a wall that the rear one is 51 mm further
        // from, so part of the raw difference is heading, not offset.
        const float corr_f = yaw_valid_
                                 ? ((float)raw_diff - chassis::STAGGER_COEFF_MM * tan_psi_)
                                 : (float)raw_diff;
        corr_diff_mm_ = (int16_t)lroundf(corr_f);

        const int abs_corr = corr_diff_mm_ < 0 ? -corr_diff_mm_ : corr_diff_mm_;
        const bool lat_ok = abs_corr <= (int)cfg_.align_diff_mm;
        const float abs_yaw = yaw_deg_ < 0.0f ? -yaw_deg_ : yaw_deg_;
        const bool yaw_ok = !yaw_valid_ || (abs_yaw <= cfg_.yaw_tol_deg);

        if (lat_ok && yaw_ok) {
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
                second_leg_ms_ = 0;
                counter_wheel_deg_ = 0.0f;
                wheel_deg_ = 0.0f;
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
        float lat_wheel = 0.0f;
        if (!lat_ok) {
            const float dt_s =
                (last_pid_ms_ == 0) ? 1.0f : (float)(now_ms - last_pid_ms_) * 0.001f;
            last_pid_ms_ = now_ms;
            lat_wheel = pid_.computeWheelDeg((float)corr_diff_mm_, dt_s, cfg_);
        } else {
            // A heading-only fix is a rotation, not a translation: carrying the
            // old integral into it would leave a lateral bias behind.
            pid_.reset();
        }

        // Nose-right is corrected by steering left while driving forward, which
        // is the positive-wheel-angle direction. The trim is split over the two
        // halves so the S-curve's rotation adds up to the full correction.
        const float yaw_wheel = yaw_valid_ ? (cfg_.k_yaw * yaw_deg_) : 0.0f;
        const float first_w =
            util::clampf(+lat_wheel + 0.5f * yaw_wheel, -steering::MAX_WHEEL_DEG,
                         steering::MAX_WHEEL_DEG);
        const float second_w =
            util::clampf(-lat_wheel + 0.5f * yaw_wheel, -steering::MAX_WHEEL_DEG,
                         steering::MAX_WHEEL_DEG);
        const float only_yaw_w = util::clampf(yaw_wheel, -steering::MAX_WHEEL_DEG,
                                              steering::MAX_WHEEL_DEG);

        if (!lat_ok) {
            // Half the stroke each way: equal time at +d and -d cancels the
            // heading the first half adds, leaving only the sideways shift.
            move_ms_ = cfg_.forward_move_ms / 2;
            second_leg_ms_ = cfg_.forward_move_ms - move_ms_;
            wheel_deg_ = first_w;
            counter_wheel_deg_ = second_w;
        } else {
            move_ms_ = cfg_.forward_move_ms / 2;
            second_leg_ms_ = 0;
            wheel_deg_ = only_yaw_w;
            counter_wheel_deg_ = 0.0f;
        }

        act.setWheelAngleDeg(wheel_deg_);
        ++cycles_;

        // The motors stay off until the wheels are where the controller asked.
        phase_start_ms_ = now_ms;
        phase_ = Phase::STEER_WAIT;
    }

    Telemetry snapshot(uint32_t now_ms, const SensorReading& s, Actuators& act) const {
        Telemetry t;
        t.t_ms = now_ms;
        t.gL_mm = s.gL_mm;
        t.gR_mm = s.gR_mm;
        t.raw_diff_mm = (int16_t)s.gL_mm - (int16_t)s.gR_mm;
        t.diff_mm = yaw_valid_ ? corr_diff_mm_ : t.raw_diff_mm;
        t.rear_mm = s.rear_mm;
        t.rear_target_mm = cfg_.rear_target_mm;
        t.align_diff_mm = cfg_.align_diff_mm;
        t.backoff_max = cfg_.backoff_max;
        t.duty = cfg_.duty;
        t.steer_settle_ms = (uint16_t)cfg_.steer_settle_ms;
        t.servo_deg = (int16_t)act.servoDeg();
        t.wheel_deg = wheel_deg_;
        t.yaw_deg = yaw_deg_;
        t.yaw_tol_deg = cfg_.yaw_tol_deg;
        t.k_yaw = cfg_.k_yaw;
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
    // Length of the counter-steered second half; zero means single stroke.
    uint32_t second_leg_ms_ = 0;
    float counter_wheel_deg_ = 0.0f;
    // Reverse-leg anchor for the yaw slope.
    uint16_t rev_start_rear_mm_ = 0;
    int16_t rev_start_diff_mm_ = 0;
    bool rev_start_valid_ = false;
    float tan_psi_ = 0.0f;
    float yaw_deg_ = 0.0f;
    bool yaw_valid_ = false;
    int16_t corr_diff_mm_ = 0;
    uint16_t backoffs_ = 0;
    float wheel_deg_ = 0.0f;
    uint16_t cycles_ = 0;
    const char* fault_ = "none";
};
