#pragma once

#include <cmath>

#include "config.h"

// Ackermann steering referred to the rear axle. R = L_wb / tan(delta_eq) is exact because the instant centre lies there. The split across the front wheels uses cot(d_out) - cot(d_in) = t_front / L_wb, and swapping the two gives a radius that never varies.

namespace ack {

// Turn radius of the rear axle centre in metres, capped on the straight.
inline float turnRadius(const Config&, float deq) {
  const float s = tanf(fabsf(deq));
  if (s <= 1e-6f) return 1.0e6f;
  return car::L_wb / s;
}

// Inner wheel angle in radians, about 5 degrees more lock than delta_eq at 25.
inline float deltaIn(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float r = R - 0.5f * car::t_front;
  // Impossible radius, so clamp rather than flip the sign.
  if (r <= 1e-4f) return (deq >= 0.0f) ? c.delta_max : -c.delta_max;
  const float a = atanf(car::L_wb / r);
  return (deq >= 0.0f) ? a : -a;
}

// Outer wheel angle in radians, always less lock, and it never binds.
inline float deltaOut(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float a = atanf(car::L_wb / (R + 0.5f * car::t_front));
  return (deq >= 0.0f) ? a : -a;
}

// The equivalent angle a measured pair implies. The mean is taken over the cotangents, because an angle mean reads 0.69 degrees high at full lock.
inline float deltaEq(const Config&, float din, float dout) {
  const float ci = tanf(din);
  const float co = tanf(dout);
  if (fabsf(ci) < 1e-6f || fabsf(co) < 1e-6f) return 0.0f;
  const float s = 1.0f / ci + 1.0f / co;
  if (fabsf(s) < 1e-6f) return 0.0f;
  return atanf(2.0f / s);
}

// Largest equivalent angle before the inner wheel's stop binds, about 20.2 deg.
inline float deltaEqMax(const Config& c) {
  const float lim = c.delta_max - c.servo_deadband;
  if (lim <= 1e-4f) return 0.0f;
  const float r = car::L_wb / tanf(lim) + 0.5f * car::t_front;
  if (r <= 1e-4f) return 0.0f;
  return atanf(car::L_wb / r);
}

}
