#pragma once

// Every tunable in one place. A value marked PLACEHOLDER has not been
// measured yet, and PLAN.md section 13 lists what still has to be measured.
// The bay is made by hand, so its width and depth are not in this file at all:
// they are measured at run time from the sensors (see supervisor.h).

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

  // Run-time calibration. S0 is the sensor sum with the car squared up, and
  // trim is the half difference of the two readings when the car is centred.
  // speed and d_stop are measured on the bench and are still placeholders. ---
  float S0        = 0.0f;
  float trim      = 0.0f;
  float speed     = 0.15f;
  float d_stop    = 0.020f;
  // end

  // Supervisor limits. tol_off and tol_align are the offset and heading
  // tolerances, and both get widened at run time when the measured sensor
  // noise calls for it. rear_target is the wanted gap to the rear wall,
  // rear_margin covers sensor latency and overshoot, and rear_release is the
  // gap that re-arms the brake. dwell is how many consecutive ticks a
  // transition has to hold, max_strokes is the correction budget, stroke_dir
  // is +1 to run strokes forward out of the bay, brake_settle is the wait
  // after braking, and stroke_timeout is a hard limit on one stroke. ---
  float tol_off      = 0.010f;
  float tol_align    = 0.061f;
  float rear_target  = 0.050f;
  float rear_margin  = 0.012f;
  float rear_release = 0.090f;
  int   dwell        = 3;
  int   max_strokes  = 12;
  float stroke_dir   = 1.0f;
  float brake_settle = 0.15f;
  float stroke_timeout = 3.0f;
  // end

  // Steering, all in radians unless noted. delta_max is the mechanical steer
  // limit, delta_coarse and delta_fine are the two stroke amplitudes, and
  // fine_threshold is the offset below which the fine amplitude is used.
  // psi_cap_min and psi_cap_max bound the peak-yaw cap, psi_meas_max is the
  // cone-safe limit on the excursion, servo_deadband is half the SG90
  // deadband and servo_rate is the slew rate in radians per second.
  // accel_model is the drive acceleration used for the time limits, and
  // tol_align_deg is the heading tolerance in degrees, which has to stay clear
  // of the estimate noise. ---
  float delta_max     = 0.436f;
  float delta_coarse  = 0.436f;
  float delta_fine    = 0.209f;
  float fine_threshold = 0.020f;
  float psi_cap_min   = 0.087f;
  float psi_cap_max   = 0.436f;
  float psi_meas_max  = 0.209f;
  float servo_deadband = 0.026f;
  float servo_rate    = 5.236f;
  float accel_model   = 0.60f;
  float tol_align_deg = 3.5f;
  // end

  // Inner loop. omega_n is the closed-loop natural frequency in radians per
  // second and zeta the damping ratio. ki is the outer integral gain on the
  // offset, which only trims out a steady bias, and i_max clamps that
  // integral. tau_psi and tau_x are the low-pass time constants on the heading
  // and offset estimates, tau_filt is the filter on the state used for the
  // tolerance decisions, and rotate_margin says how far past the tolerance a
  // rotate stroke is allowed to go. dt is the control period, which is the
  // sensor rate. ---
  float omega_n = 1.2f;
  float zeta    = 0.9f;
  float ki      = 0.0f;
  float i_max   = 0.20f;
  float tau_psi = 0.05f;
  float tau_filt = 0.25f;
  float rotate_margin = 1.0f;
  float tau_x   = 0.03f;
  float dt      = 0.033f;
  // end
};
