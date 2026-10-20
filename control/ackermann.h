#pragma once

#include <cmath>

#include "config.h"

// Ackermann steering geometry. See PLAN.md section 6.6.
//
// The controller works in one angle, the equivalent bicycle-model steer
// delta_eq, defined by the turn radius of the rear axle centre:
//
//   R = L_wb / tan(delta_eq)
//
// That is exact rather than approximate. An Ackermann vehicle's instant centre
// of rotation lies on the rear axle line, necessarily, because the rear wheels
// are fixed and so cannot slip sideways. This design already references the
// whole vehicle state to the rear axle centre. The bicycle plant in
// tools/sim_main.cpp is therefore exact for true Ackermann, and this module
// only decides how that one angle splits across the two front wheels. Had the
// state been referenced to the body centre or the front axle, this would have
// been a real problem.
//
// Two relations carry the geometry, and they are easy to swap. With the inner
// wheel angle d_in and the outer d_out:
//
//   Ackermann condition : cot(d_in) - cot(d_out) = t_front / L_wb    a difference
//   turn radius         : R = L_wb * (cot(d_in) + cot(d_out)) / 2    a sum
//
// The difference is constant at every lock angle. Only the sum changes with
// lock. Swapping them returns t_front/L_wb for every input, which reads like
// a radius but never varies, and nothing else looks wrong. An earlier draft
// of this design had exactly that bug.
//
// All angles are signed, positive to the left, matching control.h.

namespace ack {

// Turn radius of the rear axle centre in metres. It is unbounded going
// straight, so it is capped at a large finite value: callers subtract from it
// and divide by it, and infinities survive neither. ---
inline float turnRadius(const Config& c, float deq) {
  const float s = tanf(fabsf(deq));
  if (s <= 1e-6f) return 1.0e6f;
  return c.L_wb / s;
}
// end

// Inner wheel angle in radians. The inner wheel is the one on the side the car
// turns towards, and it always takes more lock than delta_eq. That extra is
// about 1.2 times, which is 5 degrees more at delta_eq = 25 degrees with
// L_wb = 0.18 and t_front = 0.15. ---
inline float deltaIn(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float r = R - 0.5f * c.t_front;
  // A radius of t_front/2 or less is geometrically impossible, because the
  // turn circle would sit inside the track. It cannot happen while delta_max
  // is 25 degrees, but clamp rather than return a sign-flipped angle. ---
  if (r <= 1e-4f) return (deq >= 0.0f) ? c.delta_max : -c.delta_max;
  // end
  const float a = atanf(c.L_wb / r);
  return (deq >= 0.0f) ? a : -a;
}
// end

// Outer wheel angle in radians. It always takes less lock than delta_eq, and
// it never binds, which is why the inner wheel sets the mechanical limit. ---
inline float deltaOut(const Config& c, float deq) {
  const float R = turnRadius(c, deq);
  const float a = atanf(c.L_wb / (R + 0.5f * c.t_front));
  return (deq >= 0.0f) ? a : -a;
}
// end

// The equivalent angle implied by a measured pair of wheel angles. This is the
// direction a calibration actually runs, since you measure the wheels and
// infer delta_eq rather than the other way round, so it is the one to reach
// for.
//
// The mean is of cotangents, not of the angles. The angle mean reads 0.09
// degrees high at the fine stroke amplitude and 0.69 degrees high at full
// lock, which is 3 percent of tan(delta) and shows up as a turn-radius bias.
// ---
inline float deltaEq(const Config& /*c*/, float din, float dout) {
  const float ci = tanf(din);
  const float co = tanf(dout);
  if (fabsf(ci) < 1e-6f || fabsf(co) < 1e-6f) return 0.0f;
  const float s = 1.0f / ci + 1.0f / co;  // equals 2*cot(delta_eq)
  if (fabsf(s) < 1e-6f) return 0.0f;
  return atanf(2.0f / s);
}
// end

// The largest equivalent angle the inner wheel can reach before its own
// mechanical stop binds. Because the inner wheel needs more lock than the
// equivalent angle, its stop sets the limit on delta_eq, and the result is
// well under delta_max.
//
// This is not wired in yet. control.h still clamps to c.delta_max, which
// reads as the equivalent angle and is about 5 degrees optimistic. See
// PLAN.md 6.6 and the open items. Fixing it changes the control output clamp,
// so it is left for review. ---
inline float deltaEqMax(const Config& c) {
  // The wheels reach the command less the deadband (PLAN.md 5.2). ---
  const float lim = c.delta_max - c.servo_deadband;
  if (lim <= 1e-4f) return 0.0f;
  // end

  // Invert deltaIn: lim = atan(L_wb / (R - t_front/2)), so R is
  // L_wb/tan(lim) plus the half track. ---
  const float r = c.L_wb / tanf(lim) + 0.5f * c.t_front;
  if (r <= 1e-4f) return 0.0f;
  return atanf(c.L_wb / r);
  // end
}
// end

}  // namespace ack
