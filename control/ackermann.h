#pragma once

#include <cmath>

#include "config.h"

// Ackermann steering geometry. see PLAN.md section 6.6.
//
// the controller works in one angle, the equivalent bicycle-model steer
// delta_eq, defined by the turn radius of the rear axle centre:
//
//   R = L_wb / tan(delta_eq)
//
// that is exact rather than approximate. an Ackermann vehicle's instant centre
// of rotation lies on the rear axle line, necessarily, because the rear wheels
// are fixed and so cannot slip sideways. this design already references the
// whole vehicle state to the rear axle centre. the bicycle plant in
// tools/sim_main.cpp is therefore exact for true Ackermann, and this module
// only decides how that one angle splits across the two front wheels. had the
// state been referenced to the body centre or the front axle, this would have
// been a real problem.
//
// two relations carry the geometry, and they are easy to swap. with the inner
// wheel angle d_in and the outer d_out:
//
//   Ackermann condition : cot(d_out) - cot(d_in) = t_front / L_wb    a difference
//   turn radius         : R = L_wb * (cot(d_in) + cot(d_out)) / 2    a sum
//
// the inner wheel has the smaller radius, so its cotangent is the smaller one,
// and the condition has the outer one first. the difference is constant at
// every lock angle. only the sum changes with lock. swapping them returns
// t_front/L_wb for every input, which reads like a radius but never varies, and
// nothing else looks wrong. an earlier draft of this design had exactly that
// bug.
//
// all angles are signed, positive to the left, matching control.h.

namespace ack {

// turn radius of the rear axle centre in metres. it is unbounded going
// straight, so it is capped at a large finite value: callers subtract from it
// and divide by it, and infinities survive neither. ---
inline float turnRadius(const Config& , float deq) {
  const float s = tanf(fabsf(deq));
  if (s <= 1e-6f) return 1.0e6f;
  return car::L_wb / s;
}
// end

// inner wheel angle in radians. the inner wheel is the one on the side the car
// turns towards, and it always takes more lock than delta_eq: at a 25 degree
// equivalent angle the inner wheel takes about 5 degrees more. ---
inline float deltaIn(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float r = R - 0.5f * car::t_front;
  // a radius of t_front/2 or less is geometrically impossible, because the turn
  // circle would sit inside the track. it cannot happen while delta_max is 25
  // degrees, but clamp rather than return a sign-flipped angle. ---
  if (r <= 1e-4f) return (deq >= 0.0f) ? c.delta_max : -c.delta_max;
  // end
  const float a = atanf(car::L_wb / r);
  return (deq >= 0.0f) ? a : -a;
}
// end

// outer wheel angle in radians. it always takes less lock than delta_eq, and it
// never binds, which is why the inner wheel sets the mechanical limit. ---
inline float deltaOut(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float a = atanf(car::L_wb / (R + 0.5f * car::t_front));
  return (deq >= 0.0f) ? a : -a;
}
// end

// the equivalent angle implied by a measured pair of wheel angles. this is the
// direction a calibration actually runs: you measure the wheels and infer
// delta_eq rather than the other way round, so it is the one to reach for.
//
// the mean is of cotangents, not of the angles. the angle mean reads 0.09
// degrees high at the fine stroke amplitude and 0.69 degrees high at full lock,
// which is 3 percent of tan(delta) and shows up as a turn-radius bias.
// ---
inline float deltaEq(const Config& , float din, float dout) {
  const float ci = tanf(din);
  const float co = tanf(dout);
  if (fabsf(ci) < 1e-6f || fabsf(co) < 1e-6f) return 0.0f;
  // the sum of the cotangents is twice cot(delta_eq). ---
  const float s = 1.0f / ci + 1.0f / co;
  // end
  if (fabsf(s) < 1e-6f) return 0.0f;
  return atanf(2.0f / s);
}
// end

// the largest equivalent angle the inner wheel can reach before its own
// mechanical stop binds. because the inner wheel needs more lock than the
// equivalent angle, its stop sets the limit on delta_eq, and the result is well
// under delta_max.
//
// control.h clamps to this when the wheels are Ackermann steered, and to
// delta_max for parallel steering, where both wheels share one angle and the
// full lock is available. ---
inline float deltaEqMax(const Config& c) {
  // the wheels reach the command less the deadband (PLAN.md 5.2). ---
  const float lim = c.delta_max - c.servo_deadband;
  if (lim <= 1e-4f) return 0.0f;
  // end

  // invert deltaIn: lim = atan(L_wb / (R - t_front/2)), so R is
  // L_wb/tan(lim) plus the half track. ---
  const float r = car::L_wb / tanf(lim) + 0.5f * car::t_front;
  if (r <= 1e-4f) return 0.0f;
  return atanf(car::L_wb / r);
  // end
}
// end

}
