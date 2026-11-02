#pragma once

#include <cmath>

#include "config.h"

// Sensor-to-state geometry (PLAN.md section 4). The side readings are gL = W/2 + x + d_L*sin(psi) - e_L*cos(psi) and gR = W/2 - x + d_R*sin(psi) - e_R*cos(psi), so the difference carries the offset and the sum the heading, against a calibrated S0 and trim taken with the car squared up. d_L and d_R are signed as config.h describes them, and d_L + d_R is the signed separation of the two faces, not a distance behind the axle.

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

  // The divisor is the signed separation, so a right face ahead of the left
  // gives a negative sum. Only its magnitude has to be usable, because the sign
  // cancels against the sum's own sign, and the heading keeps its meaning.
  const float dsum = car::d_L + car::d_R;
  if (fabsf(dsum) < 0.005f) return p;

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

// Body-to-wall clearance on the tighter side. An unusable reading comes back
// negative, and the dashboard reports it as unknown.
inline float clearance(const Config&, float gL, float gR) {
  if (!validRange(gL) || !validRange(gR)) return -1.0f;
  float left = gL - car::e_out_L;
  float right = gR - car::e_out_R;
  if (left < right) return left;
  return right;
}
// end

}
