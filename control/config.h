#pragma once

// Every tunable in one place. Values marked PLACEHOLDER are not measured yet;
// see PLAN.md section 13 (open items). The bay itself is made by hand and its
// dimensions are not known at design time, so bay width and depth are NOT
// here: they are measured at run time from the sensors (see supervisor.h).

struct Config {
  // ---- chassis and mounts -------------------------------------------------
  float L_wb     = 0.18f;   // wheelbase [m]                              PLACEHOLDER
  float L_car    = 0.25f;   // car length [m]                             PLACEHOLDER
  float d_L      = 0.10f;   // left sensor, ahead of rear axle [m]        measured
  float d_R      = 0.10f;   // right sensor, behind rear axle [m]         measured
  float e_L      = 0.00f;   // left sensor lateral offset, + = left of centreline [m]  PLACEHOLDER
  float e_R      = 0.00f;   // right sensor lateral offset, + = right of centreline [m] PLACEHOLDER
  float e_out_L  = 0.010f;  // left sensor outboard of the body side [m]   PLACEHOLDER
  float e_out_R  = 0.010f;  // right sensor outboard of the body side [m]  PLACEHOLDER

  // ---- run-time calibration ------------------------------------------------
  float S0        = 0.0f;   // sensor sum with the car squared up [m]
  float trim      = 0.0f;   // zero-offset trim: (gL-gR)/2 when centred [m]
  float speed     = 0.15f;  // measured travel speed [m/s]                PLACEHOLDER
  float d_stop    = 0.020f; // measured braking distance [m]             PLACEHOLDER

  // ---- supervisor ----------------------------------------------------------
  float tol_off      = 0.010f;  // |x| tolerance [m]
  float tol_align    = 0.061f;  // |psi| tolerance [rad] (~3.5 deg)
  float rear_target  = 0.050f;  // wanted final gap to the rear wall [m]
  float rear_margin  = 0.012f;  // sensor latency + overshoot margin [m]
  float rear_release = 0.090f;  // gap that re-arms the brake [m]
  int   dwell        = 3;       // consecutive ticks a transition must hold
  int   max_strokes  = 12;      // corrections before giving up
  float stroke_dir   = 1.0f;    // +1 run strokes forward (out of the bay)
  float brake_settle = 0.15f;   // wait after braking before a stroke [s]
  float stroke_timeout = 3.0f;  // hard limit on one stroke [s]

  // ---- steering ------------------------------------------------------------
  float delta_max     = 0.436f; // 25 deg, mechanical steer limit [rad]
  float delta_coarse  = 0.436f; // coarse stroke amplitude [rad]
  float delta_fine    = 0.209f; // fine stroke amplitude [rad]
  float fine_threshold = 0.020f; // |x| below which the fine amplitude is used [m]
  float psi_cap_min   = 0.087f; // 5 deg, floor on the peak-yaw cap [rad]
  float psi_cap_max   = 0.436f; // 25 deg, ceiling on the peak-yaw cap [rad]
  float psi_meas_max  = 0.209f; // 12 deg, cone-safe limit for the excursion
  float servo_deadband = 0.026f; // 1.5 deg, half the SG90 deadband [rad]
  float servo_rate    = 5.236f;  // 300 deg/s, servo slew rate [rad/s]
  float accel_model   = 0.60f;   // drive acceleration used for time limits [m/s^2]
  float tol_align_deg = 3.5f;    // heading tolerance, must clear the estimate noise

  // ---- inner loop ----------------------------------------------------------
  float omega_n = 1.2f;   // closed-loop natural frequency [rad/s]
  float zeta    = 0.9f;   // damping ratio
  float ki      = 0.0f;   // outer integral gain on x (trim-bias only)
  float i_max   = 0.20f;  // integral clamp [m*s]
  float tau_psi = 0.05f;  // heading estimate low-pass [s]
  float tau_filt = 0.25f;  // filter on the state used for tolerances [s]
  float rotate_margin = 1.0f; // rotate past this multiple of the tolerance
  float tau_x   = 0.03f;  // offset estimate low-pass [s]
  float dt      = 0.033f; // control period, the sensor rate [s]
};
