#pragma once

#include <cmath>

#include "config.h"

// Sensor-to-state geometry (PLAN.md section 4). The side readings are gL = W/2 + x + d_L*sin(psi) - e_L*cos(psi) and gR = W/2 - x + d_R*sin(psi) - e_R*cos(psi), so the difference carries the offset and the sum the heading, against a calibrated S0 and trim taken with the car squared up.

namespace geom {

constexpr float kPi = 3.14159265f;

// The VL53L0X reports 8190 with no target and 65535 on a timeout.
inline bool validRange(float m) { return m > 0.0f && m < 8.0f; }

// Rear axle state the side readings resolve to. Offset in metres, heading in radians, and ok says the pair was usable.
struct Pose {
  float x = 0.0f;
  float psi = 0.0f;
  bool ok = false;
};

// Turn a pair of side readings into the state estimate.
inline Pose pose(const Config& c, float gL, float gR) {
  Pose p;
  if (!validRange(gL) || !validRange(gR)) return p;
  if (car::d_L + car::d_R <= 0.0f) return p;

  const float dsum = car::d_L + car::d_R;
  const float psi = (gL + gR - c.S0) / dsum;

  // Undo the fore/aft asymmetry leaking into the offset, then the trim.
  const float x = 0.5f * (gL - gR)
                - 0.5f * (car::d_L - car::d_R) * sinf(psi)
                - c.trim;

  p.x = x;
  p.psi = psi;
  p.ok = true;
  return p;
}

// Body-to-wall clearance on the tighter side. An unusable reading comes back negative, which tells the caller to hold the car back.
inline float clearance(const Config&, float gL, float gR) {
  if (!validRange(gL) || !validRange(gR)) return -1.0f;
  float left = gL - car::e_out_L;
  float right = gR - car::e_out_R;
  if (left < right) return left;
  return right;
}

// Peak yaw before a corner touches a wall, from the (L_car/2)*sin(psi) sweep. An unknown clearance fails closed to the small end.
inline float peakYawCap(const Config& c, float clear) {
  if (clear <= 0.0f) return c.psi_cap_min;
  float s = clear / (0.5f * car::L_car);
  if (s > 1.0f) s = 1.0f;
  float cap = asinf(s);
  if (cap < c.psi_cap_min) cap = c.psi_cap_min;
  if (cap > c.psi_cap_max) cap = c.psi_cap_max;
  return cap;
}

}
