#pragma once

// Every tunable in one place. A value marked PLACEHOLDER has not been
// measured yet, and PLAN.md section 13 lists what still has to be measured.
//
// The car's own geometry is not in the struct. It has been measured, so it is a
// constant in the `car` namespace below: the shape of the car is a fact, not a
// setting. What stays here is everything that changes between runs or between
// builds, which is the driver, the pack, the servo, the bay and the gains.
//
// The bay is made by hand, so its width and depth are not in this file at all:
// they are measured at run time from the sensors (see supervisor.h).

// How the front wheels are steered. The chassis has not been measured, so
// this is a switch. The geometry is the same either way; the limit is not,
// because with Ackermann steering the inner wheel reaches its stop first. ---
enum class SteerModel {
  PARALLEL,   // both front wheels at the same angle
  ACKERMANN,  // the wheels follow the Ackermann relation
};
// end

// The motor driver. Both drivers have the same truth table, so the firmware
// does not branch on it. What differs is the voltage the driver drops, and
// that is the speed the car does. ---
enum class MotorDriver {
  DRV8833,
  LM298,
};
// end

// What the driver does to the drive: the voltage it drops at the drive
// current, and the deceleration it brakes at. The drops are datasheet
// figures at about an amp, the brake figures are placeholders. ---
struct DriveModel {
  float drop;        // V
  float brakeDecel;  // m/s^2
};

inline DriveModel driveModel(MotorDriver d) {
  if (d == MotorDriver::LM298) return DriveModel{2.5f, 0.6f};
  return DriveModel{0.5f, 1.0f};
}
// end

// The car's measured geometry, all in metres, measured on 2026-10-24. These
// are facts about the chassis and the sensor mounts rather than tuning values,
// so they are constants. Nothing at run time should be able to change the
// shape of the car, and a placeholder here would be a mistake rather than a
// default.
//
// L_wb is the wheelbase and t_front the front track. L_car is twice the front
// overhang measured from the rear axle centre, because the yaw cap uses half
// of it as the lever from the rotation centre to the body's far corner, and
// on this chassis the front corner is the far one.
//
// d_L and d_R are the side sensors' fore and aft stations from the rear axle
// centre, left ahead and right behind. The pair measured 24cm apart, split 17cm
// ahead and 7cm behind, which is longer than the 20cm the design assumed. The
// heading estimate divides by their sum, so this is the number that sets the
// heading accuracy, and a longer baseline buys accuracy linearly: this one is
// 20 percent longer than the design, so every heading reading is 20 percent
// sharper than the plan predicted.
//
// e_out_L and e_out_R are how far the body's widest point sits outboard of
// each sensor's axis. They turn a gap reading into a body clearance. Both
// sensors sit well inboard of the shell, so these are large, and a gap reading
// smaller than them would mean the body is inside the wall. ---
namespace car {
constexpr float L_wb    = 0.208779f;
constexpr float t_front = 0.1847f;
constexpr float L_car   = 0.468f;
constexpr float d_L     = 0.17f;
constexpr float d_R     = 0.07f;
constexpr float e_out_L = 0.05797f;
constexpr float e_out_R = 0.05797f;
}  // namespace car
// end

struct Config {
  // Side sensor lateral offsets, in metres, positive away from the centreline.
  // The controller never reads these: the calibration folds their sum into S0
  // and their difference into the trim, so a mount error of a couple of
  // millimetres cannot bias the pose. Nothing has measured them, so they stay
  // at zero, and they are the one part of the mount geometry that is not a
  // constant in `car`. ---
  float e_L      = 0.00f;
  float e_R      = 0.00f;
  // end

  // The steering and the drive. Both are switches until the chassis and the
  // driver have been measured. ---
  SteerModel steer_model = SteerModel::PARALLEL;
  MotorDriver motor_driver = MotorDriver::DRV8833;
  // end

  // The supply the motor runs on. A placeholder until the pack is known. ---
  float motor_supply = 7.4f;
  // end

  // How much the front wheels drag in a turn when they are not steered to
  // the Ackermann relation. ---
  float scrub_gain = 0.05f;
  // end

  // Run-time calibration. S0 is the sensor sum with the car squared up, and
  // trim is the half difference of the two readings when the car is centred.
  // calib_samples readings are averaged, because one carries the whole sensor
  // noise into the estimate for the rest of the run. ---
  float S0        = 0.0f;
  float trim      = 0.0f;
  int   calib_samples = 32;
  float speed     = 0.15f;
  float d_stop    = 0.020f;
  // end

  // Supervisor limits. tol_off and tol_align are the offset and heading
  // tolerances, and both get widened at run time when the measured sensor
  // noise calls for it. rear_target is the wanted gap to the rear wall and
  // rear_margin covers sensor latency and overshoot. dwell is how many
  // consecutive ticks a transition has to hold, max_strokes is the correction
  // budget, stroke_dir is +1 to run strokes forward out of the bay,
  // brake_settle is the wait after braking, and stroke_timeout is a hard
  // limit on one stroke. ---
  float tol_off      = 0.010f;
  float tol_align    = 0.061f;
  float rear_target  = 0.050f;
  float rear_margin  = 0.012f;
  int   dwell        = 3;
  int   max_strokes  = 12;
  float stroke_dir   = 1.0f;
  float brake_settle = 0.15f;
  float stroke_timeout = 3.0f;
  // end

  // The watchdogs. The rear sensor is the only thing between the car and the
  // wall, so an unusable rear reading stops the car. back_in_max_travel is
  // the furthest the car may reverse, estimated from the commanded speed
  // because there is no odometer, and it is the backstop for a rear reading
  // that is usable but wrong. ---
  int   rear_fault_ticks = 3;
  float back_in_max_travel = 2.5f;
  int   stroke_invalid_ticks = 2;
  // end

  // Steering, all in radians unless noted. delta_max is the mechanical steer
  // limit, delta_coarse and delta_fine are the two stroke amplitudes, and
  // fine_threshold is the offset below which the fine amplitude is used.
  // psi_cap_min and psi_cap_max bound the peak-yaw cap, psi_meas_max is the
  // cone-safe limit on the excursion, servo_deadband is half the SG90
  // deadband and servo_rate is the slew rate in radians per second. ---
  float delta_max     = 0.436f;
  float delta_coarse  = 0.436f;
  float delta_fine    = 0.209f;
  float fine_threshold = 0.020f;
  float psi_cap_min   = 0.087f;
  float psi_cap_max   = 0.436f;
  float psi_meas_max  = 0.209f;
  float servo_deadband = 0.026f;
  float servo_rate    = 5.236f;
  // end

  // Inner loop. omega_n is the closed-loop natural frequency in radians per
  // second and zeta the damping ratio. ki is the outer integral gain on the
  // offset, which only trims out a steady bias, and i_max clamps that
  // integral. tau_psi and tau_x are the low-pass time constants on the heading
  // and offset estimates, tau_filt is the filter on the state used for the
  // tolerance decisions, and dt is the control period, which is the sensor
  // rate. ---
  float omega_n = 1.2f;
  float zeta    = 0.9f;
  float ki      = 0.0f;
  float i_max   = 0.20f;
  float tau_psi = 0.05f;
  float tau_filt = 0.25f;
  float tau_x   = 0.03f;
  float dt      = 0.033f;
  // end
};

// The speed the car does with the driver that is fitted, as a fraction of
// the speed it does with the DRV8833 it was measured with. A DC motor's
// speed follows the voltage across it, and the driver takes its drop off the
// supply, so the LM298 gives the motor less of the pack than the DRV8833
// does. The speed in the config is what the car does with the DRV8833. ---
inline float driveScale(const Config& c) {
  const float fitted = c.motor_supply - driveModel(c.motor_driver).drop;
  const float ref = c.motor_supply - driveModel(MotorDriver::DRV8833).drop;
  if (fitted <= 0.0f || ref <= 0.0f) return 0.0f;
  return fitted / ref;
}
// end

// The speed the car does, with the driver that is fitted taken into account.
inline float driveSpeed(const Config& c) {
  return c.speed * driveScale(c);
}
// end
