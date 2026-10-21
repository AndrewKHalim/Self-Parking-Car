#pragma once

// Every tunable in one place. A value marked PLACEHOLDER has not been
// measured yet, and PLAN.md section 13 lists what still has to be measured.
// The bay is made by hand, so its width and depth are not in this file at all:
// they are measured at run time from the sensors (see supervisor.h).

// How the front wheels are steered. The chassis has not been measured, so
// this is a switch and not a fact. The controller works in the equivalent
// angle either way, because the bicycle model is exact for the turn radius
// and both kinds of steering give the same radius. What the switch changes
// is the limit: with Ackermann steering the inner wheel takes more lock than
// the equivalent angle, so it reaches its own stop first and the equivalent
// angle is held below what the wheel can do. ---
enum class SteerModel {
  PARALLEL,   // both front wheels at the same angle
  ACKERMANN,  // the wheels follow the Ackermann relation
};
// end

// The motor driver. The board was built for a DRV8833 and an LM298 is being
// discussed. The truth table is the same for both, so the firmware does not
// branch on it: one pin high and the other low drives, both high brakes fast,
// both low brakes slowly. What differs is the voltage the driver drops, and
// that is the speed the car does. ---
enum class MotorDriver {
  DRV8833,
  LM298,
};
// end

// What the driver does to the drive: the voltage it drops at the drive
// current, and the deceleration it brakes at. The drops are the datasheet
// figures at about one amp. The brake figures are placeholders, because
// neither driver has been measured on this car, and the stopping distance
// is what has to come off the bench. ---
struct DriveModel {
  float drop;        // V
  float brakeDecel;  // m/s^2
};

inline DriveModel driveModel(MotorDriver d) {
  if (d == MotorDriver::LM298) return DriveModel{2.5f, 0.6f};
  return DriveModel{0.5f, 1.0f};
}
// end

struct Config {
  // Chassis and mounts, all in metres. L_wb is the wheelbase, L_car the car
  // length and t_front the front track width. All three are placeholders.
  // t_front sets the inner and outer split and the delta_eq limit, and only
  // ackermann.h needs it. ---
  float L_wb     = 0.18f;
  float L_car    = 0.25f;
  float t_front  = 0.15f;
  // end

  // Side sensor mounts, in metres. d_L is the left sensor ahead of the rear
  // axle and d_R the right sensor behind it, so the pair separates heading
  // from lateral offset. Both of these have been measured. ---
  float d_L      = 0.10f;
  float d_R      = 0.10f;
  // end

  // Side sensor lateral offsets, in metres, and how far each one sits outboard
  // of the body side. The offsets are positive away from the centreline, and
  // e_out_L and e_out_R are what turn a gap into a body clearance. All four
  // are placeholders. ---
  float e_L      = 0.00f;
  float e_R      = 0.00f;
  float e_out_L  = 0.010f;
  float e_out_R  = 0.010f;
  // end

  // The steering and the drive. Both are switches, because neither the
  // chassis nor the driver has been measured, and the README is not
  // evidence. ---
  SteerModel steer_model = SteerModel::PARALLEL;
  MotorDriver motor_driver = MotorDriver::DRV8833;
  // end

  // The supply the motor runs on, and how much of it the driver takes. A
  // DC motor's speed follows the voltage across it, so the LM298 runs the
  // car at about two thirds of the speed the DRV8833 does on the same pack.
  // The supply is a placeholder until the pack is known. ---
  float motor_supply = 7.4f;
  // end

  // How much the front wheels drag in a turn when they are not steered to
  // the Ackermann relation. It is a rolling-resistance-like term, and on a
  // car this size it is worth well under a percent of the speed, which is
  // the point of measuring it rather than assuming it matters. ---
  float scrub_gain = 0.05f;
  // end

  // Run-time calibration. S0 is the sensor sum with the car squared up, and
  // trim is the half difference of the two readings when the car is centred.
  // calib_samples is how many readings are averaged to get them, because a
  // single reading carries the whole sensor noise into the estimate for the
  // rest of the run. speed and d_stop are measured on the bench and are still
  // placeholders. ---
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
  // consecutive ticks a
  // transition has to hold, max_strokes is the correction budget, stroke_dir
  // is +1 to run strokes forward out of the bay, brake_settle is the wait
  // after braking, and stroke_timeout is a hard limit on one stroke. ---
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
  // wall, so an unusable rear reading stops the car after rear_fault_ticks
  // ticks instead of letting it reverse blind. back_in_max_travel is the
  // furthest the car may reverse in the whole manoeuvre, estimated from the
  // commanded speed because there is no odometer, and it is the backstop for
  // a rear reading that is valid but wrong. stroke_invalid_ticks is how many
  // consecutive unusable side readings end a stroke. ---
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
