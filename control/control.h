#pragma once

#include <cmath>

#include "ackermann.h"
#include "config.h"

// the steering loop.
//
// the plant is close to a double integrator: steering angle turns into yaw
// rate, yaw rate turns into heading, and heading turns into lateral position.
// feedback on the heading sits one integrator away from the steering angle and
// is much better conditioned, so the structure is a cascade with the heading
// loop inside:
//
//   outer  psi_ref = -sgn(v) * (Kx * x + Ki * integral(x))
//   inner  delta   =  Kg * sgn(v) * (psi_ref - psi)
//
// expanded, that is state feedback: delta = -k1*x - k2*sgn(v)*psi, with
// characteristic polynomial s^2 + (v*k2/L)*s + (v^2*k1/L) = 0. the plant gain
// goes as v squared, so k1 and k2 get recomputed from (omega_n, zeta) at every
// update rather than fixed. that keeps the damping ratio honest as the battery
// sags.
//
// three sign rules matter, and the external review settled them. the x term
// does not flip with the direction of travel. the heading term does, because it
// needs sgn(v). the integral is frozen when the car is stopped, since the plant
// gain is zero there and the integrator would wind up during every pause.

struct Control {
  const Config* c = nullptr;

  float integ = 0.0f;
  float lpfPsi = 0.0f;
  float lpfX = 0.0f;
  bool primed = false;

  void reset() {
    integ = 0.0f;
    primed = false;
  }

  // returns the commanded steer angle at the wheel in radians, positive to the
  // left. psiCap is the peak heading excursion the current bay clearance
  // allows. xRef is the lateral reference, and zero is the parking case.
  float step(float x, float psi, float vSigned, float dt, float psiCap,
             float xRef = 0.0f) {
    if (c == nullptr) return 0.0f;

    const float v = fabsf(vSigned);

    // standstill: the plant gain is zero, so freeze the integrator and hold the
    // wheels still rather than chasing a stale estimate. ---
    if (v < 0.02f) {
      integ = 0.0f;
      lpfPsi = psi;
      lpfX = x;
      primed = true;
      return 0.0f;
    }
    // end

    // the heading estimate is the noisiest signal in the system, running one to
    // two degrees either way. it is also the only thing the fast loop acts on,
    // so it gets the heavier filter. ---
    if (!primed) {
      lpfPsi = psi;
      lpfX = x;
      primed = true;
    } else {
      float aPsi = 1.0f - expf(-dt / c->tau_psi);
      float aX = 1.0f - expf(-dt / c->tau_x);
      lpfPsi += aPsi * (psi - lpfPsi);
      lpfX += aX * (x - lpfX);
    }
    // end

    const float sgn = (vSigned >= 0.0f) ? 1.0f : -1.0f;

    // gains from (omega_n, zeta) and the current speed. kg is the inner gain
    // and kx the outer gain. ---
    const float kg = 2.0f * c->zeta * c->omega_n * car::L_wb / v;
    const float kx = c->omega_n / (2.0f * c->zeta * v);
    // end

    // outer loop: where should the car point to come back to the centreline?
    // ---
    float psiRef = -sgn * (kx * (lpfX - xRef) + c->ki * integ);
    // end

    // clamp the heading excursion to what the clearance allows. beyond this the
    // car is asking for a manoeuvre the bay cannot hold. ---
    if (psiRef > psiCap) psiRef = psiCap;
    if (psiRef < -psiCap) psiRef = -psiCap;
    // end

    // conditional integration: stop integrating while the request is clipped,
    // otherwise the integrator winds up against the clamp. ---
    const bool clipped = (psiRef >= psiCap || psiRef <= -psiCap);
    if (!clipped) {
      integ += (lpfX - xRef) * dt;
      if (integ > c->i_max) integ = c->i_max;
      if (integ < -c->i_max) integ = -c->i_max;
    }
    // end

    // inner loop: hold the heading. the sign follows the direction of travel
    // because the yaw rate is -(v/L)*delta, so the same steering angle turns
    // the car the other way when it is going backwards. ---
    float delta = -kg * sgn * (psiRef - lpfPsi);

    // the steering limit. with Ackermann steering the inner wheel reaches its
    // own stop first, so the equivalent angle is held below what the wheel can
    // deliver. ---
    const float limit = (c->steer_model == SteerModel::ACKERMANN)
                            ? ack::deltaEqMax(*c)
                            : c->delta_max;
    if (delta > limit) delta = limit;
    if (delta < -limit) delta = -limit;
    return delta;
    // end
  }
};
