#pragma once

#include <cmath>

#include "config.h"

// The steering loop.
//
// The plant is close to a double integrator: delta -> yaw rate -> heading ->
// lateral position. Feedback on heading is one integrator from the steering
// angle and is far better conditioned, so the structure is a cascade with the
// heading loop inside:
//
//   outer  psi_ref = -sgn(v) * (Kx * x + Ki * integral(x))
//   inner  delta   =  Kg * sgn(v) * (psi_ref - psi)
//
// Expanding that gives state feedback, delta = -k1*x - k2*sgn(v)*psi, whose
// characteristic polynomial is s^2 + (v*k2/L)*s + (v^2*k1/L) = 0. The plant
// gain goes as v^2, so k1 and k2 are recomputed from (omega_n, zeta) at every
// update instead of being fixed numbers. That keeps the damping ratio honest
// as the battery sags.
//
// The sign rules matter and are the ones the external review settled:
//   * the x term does NOT flip with the direction of travel,
//   * the heading term DOES (it needs sgn(v)),
//   * the integral is frozen when the car is stopped, because the plant gain
//     is zero there and the integrator would wind up during every pause.

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

  // Returns the commanded steer angle at the wheel [rad], positive = left.
  // psiCap is the peak heading excursion the current bay clearance allows.
  float step(float x, float psi, float vSigned, float dt, float psiCap) {
    if (c == nullptr) return 0.0f;

    const float v = fabsf(vSigned);
    if (v < 0.02f) {
      // Standstill: the plant gain is zero, so freeze the integrator and hold
      // the wheels still rather than chasing a stale estimate.
      integ = 0.0f;
      lpfPsi = psi;
      lpfX = x;
      primed = true;
      return 0.0f;
    }

    if (!primed) {
      lpfPsi = psi;
      lpfX = x;
      primed = true;
    } else {
      // The heading estimate is the noisiest signal in the system (+/- 1 to
      // 2 deg). It is the only thing the fast loop acts on, so it gets the
      // heavier filter.
      float aPsi = 1.0f - expf(-dt / c->tau_psi);
      float aX = 1.0f - expf(-dt / c->tau_x);
      lpfPsi += aPsi * (psi - lpfPsi);
      lpfX += aX * (x - lpfX);
    }

    const float sgn = (vSigned >= 0.0f) ? 1.0f : -1.0f;

    // Gains from (omega_n, zeta) and the current speed.
    const float kg = 2.0f * c->zeta * c->omega_n * c->L_wb / v;   // inner gain
    const float kx = c->omega_n / (2.0f * c->zeta * v);            // outer gain

    // Outer loop: where should the car point to come back to the centreline?
    float psiRef = -sgn * (kx * lpfX + c->ki * integ);

    // Clamp the heading excursion to what the clearance allows. Beyond this
    // the car is asking for a manoeuvre the bay cannot hold.
    if (psiRef > psiCap) psiRef = psiCap;
    if (psiRef < -psiCap) psiRef = -psiCap;

    // Conditional integration: stop integrating while the request is clipped,
    // otherwise the integrator winds up against the clamp.
    if (psiRef >= psiCap || psiRef <= -psiCap) {
      // frozen this cycle
    } else {
      integ += lpfX * dt;
      if (integ > c->i_max) integ = c->i_max;
      if (integ < -c->i_max) integ = -c->i_max;
    }

    // Inner loop: hold the heading. The sign follows the direction of travel
    // because the yaw rate is -(v/L)*delta: the same steering angle rotates
    // the car the other way when it is going backwards.
    float delta = -kg * sgn * (psiRef - lpfPsi);

    if (delta > c->delta_max) delta = c->delta_max;
    if (delta < -c->delta_max) delta = -c->delta_max;
    return delta;
  }
};
