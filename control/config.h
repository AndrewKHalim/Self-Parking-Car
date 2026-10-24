#pragma once

// Every tunable in one place. PLAN.md section 13 lists what is left to measure. The car's measured geometry is a fact, so it lives in the car namespace.

// How the front wheels are steered. An Ackermann inner wheel reaches its stop first, where PARALLEL holds both at one angle.
enum class SteerModel {
  PARALLEL,
  ACKERMANN,
};

// The motor driver. Same truth table either way, only the dropout differs.
enum class MotorDriver {
  DRV8833,
  LM298,
};

// Volts dropped at about an amp, and the deceleration it brakes at in m/s^2.
struct DriveModel {
  float drop;
  float brakeDecel;
};

inline DriveModel driveModel(MotorDriver d) {
  if (d == MotorDriver::LM298) return DriveModel{2.5f, 0.6f};
  return DriveModel{0.5f, 1.0f};
}

// Measured 2026-10-24, in metres. d_L and d_R are 17cm ahead and 7cm behind, a 24cm baseline where the design assumed 20, so headings read 20 percent sharper. e_out_L and e_out_R turn a gap reading into a body clearance.
namespace car {
constexpr float L_wb    = 0.208779f;
constexpr float t_front = 0.1847f;
constexpr float L_car   = 0.468f;
constexpr float d_L     = 0.17f;
constexpr float d_R     = 0.07f;
constexpr float e_out_L = 0.05797f;
constexpr float e_out_R = 0.05797f;
}

struct Config {
  // Sensor offsets in metres. Unmeasured, so zero, and the calibration folds them into S0 and the trim.
  float e_L      = 0.00f;
  float e_R      = 0.00f;

  // Switches until the chassis and the driver are measured, and a placeholder pack voltage.
  SteerModel steer_model = SteerModel::PARALLEL;
  MotorDriver motor_driver = MotorDriver::DRV8833;
  float motor_supply = 7.4f;

  // Front wheel drag in a turn that is not steered to the Ackermann relation.
  float scrub_gain = 0.05f;

  // Run-time calibration. S0 is the sensor sum with the car squared up, and the trim is half the centred difference. Averaging keeps noise out of the run.
  float S0        = 0.0f;
  float trim      = 0.0f;
  int   calib_samples = 32;
  float speed     = 0.15f;
  float d_stop    = 0.020f;

  // Tolerances, bay target, correction budget and stroke timing.
  float tol_off      = 0.010f;
  float tol_align    = 0.061f;
  float rear_target  = 0.050f;
  float rear_margin  = 0.012f;
  int   dwell        = 3;
  int   max_strokes  = 12;
  float stroke_dir   = 1.0f;
  float brake_settle = 0.15f;
  float stroke_timeout = 3.0f;

  // Watchdogs. Rear readings in a row, the furthest reverse with no odometer, and unusable side readings during a stroke.
  int   rear_fault_ticks = 3;
  float back_in_max_travel = 2.5f;
  int   stroke_invalid_ticks = 2;

  // Steering in radians. Mechanical limit, stroke amplitudes, yaw caps, and the SG90 10us dead band and 0.10s/60deg speed at 4.8v.
  float delta_max     = 0.436f;
  float delta_coarse  = 0.436f;
  float delta_fine    = 0.209f;
  float fine_threshold = 0.020f;
  float psi_cap_min   = 0.087f;
  float psi_cap_max   = 0.436f;
  float psi_meas_max  = 0.209f;
  float servo_deadband = 0.0314f;
  float servo_rate    = 10.472f;

  // Closed-loop shape in rad/s and damping ratio, the outer integral and its clamp, the three filter constants, and the control period.
  float omega_n = 1.2f;
  float zeta    = 0.9f;
  float ki      = 0.0f;
  float i_max   = 0.20f;
  float tau_psi = 0.05f;
  float tau_filt = 0.25f;
  float tau_x   = 0.03f;
  float dt      = 0.033f;
};

// Speed with the fitted driver, as a fraction of the DRV8833 the car was measured with. A DC motor follows the volts across it, and the driver takes its drop.
inline float driveScale(const Config& c) {
  const float fitted = c.motor_supply - driveModel(c.motor_driver).drop;
  const float ref = c.motor_supply - driveModel(MotorDriver::DRV8833).drop;
  if (fitted <= 0.0f || ref <= 0.0f) return 0.0f;
  return fitted / ref;
}

// Speed with the fitted driver taken into account.
inline float driveSpeed(const Config& c) {
  return c.speed * driveScale(c);
}
