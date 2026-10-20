#pragma once

#include <cmath>

#include "config.h"

// Sensor-to-state geometry. See PLAN.md section 4.
//
// With the car yawed by psi and its rear axle offset by x from the bay
// centreline, the side sensors read
//
//   gL = W/2 + x + d_L*sin(psi) - e_L*cos(psi)
//   gR = W/2 - x + d_R*sin(psi) - e_R*cos(psi)
//
// so the difference isolates the offset and the sum carries the heading:
//
//   gL - gR = 2x + (d_L - d_R)*sin(psi) - (e_L - e_R)*cos(psi)
//   gL + gR = W  + (d_L + d_R)*sin(psi) - (e_L + e_R)*cos(psi)
//
// psi is recovered from the sum against a calibrated S0 taken with the car
// squared up, and x from the difference against a calibrated trim. S0 absorbs
// the bay width and both lateral mount offsets in one number; the trim
// absorbs the left/right mount asymmetry.

namespace geom {

constexpr float kPi = 3.14159265f;

// The VL53L0X reports 8190 when it sees no target and 65535 on a timeout.
// Those must be rejected, never averaged into a reading.
inline bool validRange(float m) { return m > 0.0f && m < 8.0f; }

struct Pose {
  float x = 0.0f;    // lateral offset of the rear axle from the bay centreline [m]
  float psi = 0.0f;  // heading relative to the bay lines [rad]
  bool ok = false;
};

// Convert a pair of side readings into the state estimate.
inline Pose pose(const Config& c, float gL, float gR) {
  Pose p;
  if (!validRange(gL) || !validRange(gR)) return p;
  if (c.d_L + c.d_R <= 0.0f) return p;

  const float dsum = c.d_L + c.d_R;
  // S = S0 + (d_L + d_R)*sin(psi), so psi comes out of the sum this way round.
  const float psi = (gL + gR - c.S0) / dsum;

  // Undo the fore/aft asymmetry leak into the offset, then the calibrated trim.
  const float x = 0.5f * (gL - gR)
                - 0.5f * (c.d_L - c.d_R) * sinf(psi)
                - c.trim;

  p.x = x;
  p.psi = psi;
  p.ok = true;
  return p;
}

// Body-to-wall clearance on the tighter side [m]. The side sensors read to
// the wall, so the body edge is that much closer in.
inline float clearance(const Config& c, float gL, float gR) {
  float left = (validRange(gL) ? gL : 10.0f) - c.e_out_L;
  float right = (validRange(gR) ? gR : 10.0f) - c.e_out_R;
  if (left < right) return left;
  return right;
}

// Peak yaw the body can swing before a corner touches a wall. The body sweeps
// (L_car/2)*sin(psi) sideways, so the cap follows from the clearance.
inline float peakYawCap(const Config& c, float clear) {
  if (clear <= 0.0f) return 0.0f;
  float s = clear / (0.5f * c.L_car);
  if (s > 1.0f) s = 1.0f;
  float cap = asinf(s);
  if (cap < c.psi_cap_min) cap = c.psi_cap_min;
  if (cap > c.psi_cap_max) cap = c.psi_cap_max;
  return cap;
}

}  // namespace geom
