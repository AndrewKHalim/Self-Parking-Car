#pragma once

#include <cmath>

#include "ackermann.h"
#include "config.h"

// The steering loop, a cascade with the heading loop inside. Outer psi_ref = -sgn(v) * (Kx * x + Ki * integral(x)), inner delta = Kg * sgn(v) * (psi_ref - psi). Expanded that is state feedback with plant gain v^2, so the gains are recomputed every update. x does not flip with travel direction, the heading term does, and the integral is frozen at standstill where the gain is zero.

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

  // Steer angle at the wheel in radians, positive left. psiCap is the peak heading excursion the clearance allows, xRef the lateral reference.
  float step(float x, float psi, float vSigned, float dt, float psiCap,
             float xRef = 0.0f) {
    if (c == nullptr) return 0.0f;

    const float v = fabsf(vSigned);

    // Plant gain is zero at standstill, so the integrator freezes.
    if (v < 0.02f) {
      integ = 0.0f;
      lpfPsi = psi;
      lpfX = x;
      primed = true;
      return 0.0f;
    }

    // The heading estimate is the noisiest signal, so it gets the heavier filter.
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

    const float sgn = (vSigned >= 0.0f) ? 1.0f : -1.0f;

    // Gains from (omega_n, zeta) at the current speed, kg inner and kx outer.
    const float kg = 2.0f * c->zeta * c->omega_n * car::L_wb / v;
    const float kx = c->omega_n / (2.0f * c->zeta * v);

    // Outer loop. Where the car should point to come back to the centreline.
    float psiRef = -sgn * (kx * (lpfX - xRef) + c->ki * integ);

    // Keep the excursion inside what the bay allows.
    if (psiRef > psiCap) psiRef = psiCap;
    if (psiRef < -psiCap) psiRef = -psiCap;

    // Conditional integration, or the integrator winds up against the clamp.
    const bool clipped = (psiRef >= psiCap || psiRef <= -psiCap);
    if (!clipped) {
      integ += (lpfX - xRef) * dt;
      if (integ > c->i_max) integ = c->i_max;
      if (integ < -c->i_max) integ = -c->i_max;
    }

    // Inner loop. Yaw rate is -(v/L)*delta, so the sign follows travel direction.
    float delta = -kg * sgn * (psiRef - lpfPsi);

    // Under Ackermann the inner wheel reaches its own stop first.
    const float limit = (c->steer_model == SteerModel::ACKERMANN)
                            ? ack::deltaEqMax(*c)
                            : c->delta_max;
    if (delta > limit) delta = limit;
    if (delta < -limit) delta = -limit;
    return delta;
  }
};
