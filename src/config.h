// Single source of truth for hardware pins, CAD-derived geometry, the pinion
// linkage ratio and the maneuver/PID defaults. Nothing below is measured by
// this firmware; every number comes from the CAD tree or a default the WebUI
// can change at runtime, so the code can be retuned without a reflash.
#pragma once

#include <stdint.h>

namespace util {
// Local clamps instead of Arduino's constrain() macro so the controller and
// PID headers can also be compiled on the host for the maneuver smoke test.
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
inline int clampi(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace util

namespace pins {
// ESP32-S3 Super Mini. Keep this list short: the module exposes fewer usable
// GPIOs than the devkit, and every one of them is spoken for.
constexpr int I2C_SDA = 8;
constexpr int I2C_SCL = 9;

constexpr int XSHUT_LEFT = 1;
constexpr int XSHUT_RIGHT = 2;
constexpr int XSHUT_REAR = 10;

constexpr int SERVO = 11;

constexpr int MOTOR_L_IN1 = 4;
constexpr int MOTOR_L_IN2 = 5;
constexpr int MOTOR_R_IN1 = 6;
constexpr int MOTOR_R_IN2 = 7;

// Polarity follows the wiring of the existing harness. If a wheel drives the
// wrong way, flip the matching flag instead of touching the driver code.
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Every assignment above must be unique, otherwise two peripherals fight over
// one pad and the failure looks like a dead sensor.
constexpr int ALL_GPIOS[] = {I2C_SDA,  I2C_SCL,        XSHUT_LEFT,   XSHUT_RIGHT,
                             XSHUT_REAR, SERVO,       MOTOR_L_IN1,  MOTOR_L_IN2,
                             MOTOR_R_IN1, MOTOR_R_IN2};
constexpr int ALL_GPIOS_COUNT = sizeof(ALL_GPIOS) / sizeof(ALL_GPIOS[0]);

constexpr bool allDistinct() {
    for (int i = 0; i < ALL_GPIOS_COUNT; ++i) {
        for (int j = i + 1; j < ALL_GPIOS_COUNT; ++j) {
            if (ALL_GPIOS[i] == ALL_GPIOS[j]) {
                return false;
            }
        }
    }
    return true;
}
static_assert(allDistinct(), "GPIO assignments must be pairwise distinct");
}  // namespace pins

namespace sensors {
// Three VL53L0X on one bus, separated by XSHUT bring-up in this address order.
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;

constexpr uint32_t TIMING_BUDGET_US = 33000;
constexpr uint32_t PERIOD_MS = 33;
constexpr uint16_t TIMEOUT_MS = 50;

// 8190 is the library's "no target" code and 65535 comes back on an I2C
// timeout; anything above this is treated as unusable rather than as distance.
constexpr uint16_t OUT_OF_RANGE_MM = 8000;
}  // namespace sensors

namespace chassis {
// From 3d/GEOMETRY-cad.md (Base.step). Distances are millimetres.
constexpr float LENGTH_MM = 234.0f;
constexpr float WHEELBASE_MM = 183.0f;
constexpr float KINGPIN_SPACING_MM = 90.5f;
constexpr float MAX_WIDTH_MM = 170.0f;       // 17 cm outer width across the tires
constexpr float WHEEL_DIAMETER_MM = 67.0f;   // 6.7 cm tire
constexpr float WHEEL_RADIUS_MM = 0.5f * WHEEL_DIAMETER_MM;
constexpr float WHEEL_WIDTH_MM = 25.0f;      // 2.5 cm tire thickness

// Side sensor faces, both aimed straight outboard. They are not at the same
// station: the right one sits 80 mm further forward, so gL - gR carries a
// heading term as well as the lateral offset and has to be corrected before it
// can be read as an offset.
constexpr float SIDE_SENSOR_SPAN_MM = 95.0f;
constexpr float LEFT_AHEAD_AXLE_MM = 67.0f;
constexpr float RIGHT_AHEAD_AXLE_MM = 147.0f;
constexpr float REAR_BEHIND_AXLE_MM = 38.0f;  // rear axle to rear sensor face

// Geometric centre of the 234 mm body ahead of the rear axle: 234/2 - 38.
constexpr float CENTRE_AHEAD_AXLE_MM = 0.5f * LENGTH_MM - REAR_BEHIND_AXLE_MM;
// Sum of the longitudinal sensor offsets from that centre. The 80 mm stagger
// leaves this at +56.0 mm, the coefficient that scales tan(psi) out of gL - gR.
constexpr float STAGGER_COEFF_MM =
    (LEFT_AHEAD_AXLE_MM - CENTRE_AHEAD_AXLE_MM) +
    (RIGHT_AHEAD_AXLE_MM - CENTRE_AHEAD_AXLE_MM);

// Each tire's outer sidewall protrudes (170 - 95) / 2 = 37.5 mm outboard of its
// side sensor face, which is why a tire can touch a wall the gap sensor still
// calls clear.
constexpr float WHEEL_OUTBOARD_MM = 0.5f * (MAX_WIDTH_MM - SIDE_SENSOR_SPAN_MM);
// Longitudinal lever arms from each side sensor to the tire edges the chassis
// can hang on. The rear-most tire edge sits at z = -33.5 (behind the axle), the
// front-most at z = 183 + 33.5 = 216.5, so the right sensor at z = 147 is only
// 69.5 mm from the front edge but 180.5 mm from the rear one.
constexpr float LEFT_REAR_TIRE_LEVER_MM = LEFT_AHEAD_AXLE_MM + WHEEL_RADIUS_MM;
constexpr float RIGHT_REAR_TIRE_LEVER_MM = RIGHT_AHEAD_AXLE_MM + WHEEL_RADIUS_MM;
constexpr float LEFT_FRONT_TIRE_LEVER_MM =
    (WHEELBASE_MM + WHEEL_RADIUS_MM) - LEFT_AHEAD_AXLE_MM;
constexpr float RIGHT_FRONT_TIRE_LEVER_MM =
    (WHEELBASE_MM + WHEEL_RADIUS_MM) - RIGHT_AHEAD_AXLE_MM;
}  // namespace chassis

namespace steering {
// Pinion (12 teeth) drives the rack (2.6 mm pitch); the servo horn pushes the
// Ackerman arm (14.25 mm kingpin-to-pickup). Net effect: one degree of road
// wheel needs 2.92 degrees of servo, which is why a 15.4 degree wheel angle is
// already the mechanical stop.
constexpr int PINION_TEETH = 12;
constexpr float RACK_PITCH_MM = 2.6f;
constexpr float HORN_ARM_MM = 14.25f;

constexpr float SERVO_PER_WHEEL_DEG = 2.92f;
constexpr int SERVO_CENTRE_DEG = 90;
constexpr int SERVO_MIN_DEG = 45;
constexpr int SERVO_MAX_DEG = 135;

constexpr float MAX_WHEEL_DEG = 15.4f;
static_assert(SERVO_CENTRE_DEG + MAX_WHEEL_DEG * SERVO_PER_WHEEL_DEG <= SERVO_MAX_DEG,
              "MAX_WHEEL_DEG exceeds the servo travel");
static_assert(SERVO_CENTRE_DEG - MAX_WHEEL_DEG * SERVO_PER_WHEEL_DEG >= SERVO_MIN_DEG,
              "MAX_WHEEL_DEG exceeds the servo travel");

// Flipped twice on the bench: -1 was reported as steering the wrong way, so the
// correction is negated again. This is the mapping that matches the linkage.
constexpr int STEER_SIGN = 1;
}  // namespace steering

namespace drive {
constexpr uint32_t PWM_HZ = 20000;  // above hearing on a DRV8833
constexpr uint8_t PWM_BITS = 8;
// Default duty for both legs. Bench settled between 127 (too slow to finish a
// reverse leg) and 200 (too fast for the rear band). Runtime-adjustable, which
// is also the knob to reach for when a reverse leg runs out of its timeout.
constexpr uint8_t DUTY = 200;
constexpr uint8_t DUTY_MAX = 255;
}  // namespace drive

namespace defaults {
constexpr uint16_t REAR_TARGET_MM = 100;  // 10 cm stop distance, WebUI-adjustable
constexpr uint16_t REAR_TARGET_MIN_MM = 30;
constexpr uint16_t REAR_TARGET_MAX_MM = 500;

constexpr uint16_t ALIGN_DIFF_MM = 10;  // |gL - gR| that counts as aligned
constexpr uint16_t ALIGN_DIFF_MIN_MM = 1;
constexpr uint16_t ALIGN_DIFF_MAX_MM = 200;

constexpr uint32_t SETTLE_MS = 200;      // motors braked before sampling
// Time the servo gets, with the motors braked, to reach the angle it was just
// given. 200 ms is enough for the chassis to stop, not for a 45 degree swing
// through the rack, so steering and reversing each wait this long first.
// 300 ms is the bench default; raise it on the page if the wheels are still
// swinging when the car pulls away.
constexpr uint32_t STEER_SETTLE_MS = 300;
constexpr uint32_t STEER_SETTLE_MAX_MS = 2000;
constexpr uint32_t FORWARD_MOVE_MS = 1000;
// Half of the 1 cm band the rear target is allowed. Stopping closer than this
// is treated as an overshoot, not as parked.
constexpr uint16_t REAR_TOLERANCE_MM = 5;
// At duty 200 the car coasts 10-15 mm after the brake, so commanding the brake
// this far above the stop band lands the settled chassis inside target +/- tol.
constexpr uint16_t REVERSE_BRAKE_LEAD_MM = 10;
// Straight pull-away when the tail ended up inside the tolerance: it only has to
// undo the overshoot, and then it settles straight into SETTLE_REAR instead of
// reversing at the wall again, which is what the old fixed 500 ms leg did.
constexpr uint32_t BACKOFF_MIN_MS = 120;
constexpr uint32_t BACKOFF_MAX_MS = 260;
constexpr uint32_t BACKOFF_MS_PER_MM = 5;
// A nudge that pulls further than this past the target did not work; reverse.
constexpr uint16_t REAR_NUDGE_MAX_ABOVE_MM = 25;

// Side-wall guard. The tire sidewalls stick 37.5 mm outboard of the gap
// sensors, and at yaw the wheel furthest along the car reaches the wall long
// before the sensor sees it, so the clearance is projected through tan(psi). A
// 17.5 mm tire gap is a 55 mm sensor reading at psi = 0.
constexpr float TIRE_MIN_CLEARANCE_MM = 17.5f;
// A gap has to be shrinking by this much before it counts as closing; one noisy
// pair must not brake the car.
constexpr uint16_t SIDE_SHRINK_MARGIN_MM = 4;
// Below this travel the heading estimate is noise, so only the latched angle
// from the previous leg is used.
constexpr uint32_t MIN_REVERSE_DRIVE_MS = 350;
constexpr uint16_t LIVE_SLOPE_SPAN_MM = 15;
// Above 12 deg, an S-curve (+lat then -lat) preserves the diagonal pose and
// swings one end of the car into the wall; one unsplit straightening arc
// rotates the chassis straight without translating it sideways.
constexpr float LARGE_YAW_STRAIGHTEN_DEG = 12.0f;

// Stall detector: a rear wheel hung on a wall leaves rear_mm almost unchanged
// while the motors keep pushing. 8 mm over 700 ms is slower than any bench leg.
constexpr uint16_t STALL_MIN_PROGRESS_MM = 8;
constexpr uint32_t STALL_WINDOW_MS = 700;
// A wheel that small cannot climb a wall unless it is turned hard enough.
constexpr float UNSTICK_MIN_WHEEL_DEG = 9.5f;
// How many straight pull-aways before the run gives up. 0 means keep pulling;
// the WebUI can move this without a reflash.
constexpr uint8_t BACKOFF_MAX = 3;
constexpr uint8_t BACKOFF_MAX_MAX = 20;
constexpr uint32_t REVERSE_TIMEOUT_MS = 6000;

// Heading gate: a run only counts as perpendicular when the reverse-leg slope
// says the car is within this many degrees of the rear wall's normal.
constexpr float YAW_TOL_DEG = 5.0f;
constexpr float YAW_TOL_MIN_DEG = 0.5f;
constexpr float YAW_TOL_MAX_DEG = 25.0f;
// Slope from two samples only means something once the car has actually moved;
// below this span the difference is sensor noise divided by a few millimetres.
constexpr uint16_t MIN_REVERSE_SPAN_MM = 25;
// tan(40 deg): a real bay never asks for more, and the clamp keeps one bad
// sample from swinging the stagger correction across the whole range.
constexpr float MAX_TAN_PSI = 0.84f;
constexpr uint32_t CONTROL_PERIOD_MS = 33;
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
}  // namespace defaults

namespace pid_defaults {
constexpr float KP = 0.15f;  // road-wheel degrees per mm of offset
constexpr float KI = 0.01f;  // per mm-second
constexpr float KD = 0.02f;  // per mm/s
constexpr float MAX_I_WHEEL_DEG = 5.0f;

// WebUI clamps. Zero is legal (a term switched off); the upper bounds keep a
// typo from demanding more than the servo can deliver.
constexpr float KP_MAX = 5.0f;
constexpr float KI_MAX = 1.0f;
constexpr float KD_MAX = 1.0f;

// Proportional heading trim, in road-wheel degrees per degree of yaw error.
// Split over the two halves of the S-curve, so this is the full correction.
constexpr float K_YAW = 0.60f;
constexpr float K_YAW_MAX = 3.0f;
}  // namespace pid_defaults

// Runtime-mutable tuning. Kept in one struct so the WebUI can move a value
// without any code path hardcoding "9 cm" or "1 second".
struct TuningConfig {
    uint16_t rear_target_mm = defaults::REAR_TARGET_MM;
    uint16_t align_diff_mm = defaults::ALIGN_DIFF_MM;
    uint32_t settle_ms = defaults::SETTLE_MS;
    uint32_t forward_move_ms = defaults::FORWARD_MOVE_MS;
    uint32_t steer_settle_ms = defaults::STEER_SETTLE_MS;
    uint8_t backoff_max = defaults::BACKOFF_MAX;
    uint8_t duty = drive::DUTY;
    float kp = pid_defaults::KP;
    float ki = pid_defaults::KI;
    float kd = pid_defaults::KD;
    float yaw_tol_deg = defaults::YAW_TOL_DEG;
    float k_yaw = pid_defaults::K_YAW;
};

// The car is its own access point. No router credentials: there is nothing to
// configure before the page is reachable at 192.168.4.1.
namespace net_cfg {
constexpr char AP_SSID[] = "autoparking";
constexpr char AP_PASSWORD[] = "autoparking";
}  // namespace net_cfg
