#pragma once

#include <Arduino.h>
#include <math.h>

#include "config.h"

// The pose from the two side faces ---
// Each gap is the distance from that face to the bay wall, and the faces sit at
// different stations along the car.
//
//   gL = W/2 + x + d_L*psi
//   gR = W/2 - x + d_R*psi
//
// Add them and the offset cancels.
//
//   gL + gR = W + (d_L + d_R)*psi
//
// Subtract them and the width cancels.
//
//   gL - gR = 2x + (d_L - d_R)*psi
//
// With d_L + d_R = -0.080 the sum is a heading signal once W is known, and with
// d_L - d_R = 0.214 the difference is the offset plus a heading term that comes
// out once the heading is known. That is the whole reason the pair works at
// all, the two stations give the second equation.
//
// W is captured at the start of every run with the motors off and the wheels
// centred, averaged over 32 valid pairs. S0 is the bay width only if the car
// starts square. A car that starts yawed carries that yaw as a constant bias in
// every psi reading, which shows as a psi that does not sit at zero while the
// car stands straight.
//
// The offset is not referenced to the start. At psi = 0 the difference is
// already absolute, gL - gR = 2x, so subtracting the starting difference would
// make the loop hold whatever offset the car entered with instead of centring
// it. The starting difference is printed as x0 and is a diagnostic only.
// end

struct Pose {
  float x;
  float psi;
};

// Gaps and the calibrated sum all in metres.
inline Pose poseFromGaps(float gL, float gR, float s0) {
  Pose p;
  p.psi = (gL + gR - s0) / car::D_SUM;
  p.x = 0.5f * (gL - gR) - 0.5f * car::D_DIFF * p.psi;
  return p;
}

// One exponential average per channel. The first sample seeds it, so a filter
// never starts from zero and drags the pose with it.
// end
struct Ema {
  float value = 0.0f;
  bool primed = false;

  float step(float v, float a) {
    if (!primed) {
      value = v;
      primed = true;
    } else {
      value += a * (v - value);
    }
    return value;
  }

};

struct Steer {
  float psiRef;
  int servoDeg;
};

// The cascade. The outer loop on offset gives a heading reference, the inner
// loop on heading gives a road wheel angle, and the linkage turns that into a
// servo command. The derivative runs on the filtered heading, because the raw
// one at 25 ms carries more sensor noise than motion.
// end
class Cascade {
public:
  void reset() {
    integral_ = 0.0f;
    psiF_ = 0.0f;
    psiPrev_ = 0.0f;
    primed_ = false;
  }

  // Start the filter at the measured heading, so the first derivative is a
  // difference of two real samples and not a step from zero.
  void seed(float psi) {
    psiF_ = psi;
    psiPrev_ = psi;
    primed_ = true;
  }

  Steer step(float x, float psi, float dt) {
    if (!primed_) {
      psiF_ = psi;
      psiPrev_ = psi;
      primed_ = true;
    } else {
      const float a = dt / (control::PSI_TAU_S + dt);
      psiF_ += a * (psi - psiF_);
    }

    float psiDot = (psiF_ - psiPrev_) / dt;
    psiPrev_ = psiF_;
    if (psiDot > control::PSI_DOT_MAX) psiDot = control::PSI_DOT_MAX;
    if (psiDot < -control::PSI_DOT_MAX) psiDot = -control::PSI_DOT_MAX;

    if (!control::STEER_ENABLE) return hold();

    // The deadband holds the wheels straight instead of chasing noise. Eight
    // millimetres and one degree are inside what the sensor and the linkage
    // can resolve, so a correction there is a correction on noise. Nothing is
    // reset, so the loops resume from where they were when the car drifts out.
    if (fabsf(x) <= control::X_DEADBAND_M &&
        fabsf(psiF_) <= control::PSI_DEADBAND_RAD) {
      return hold();
    }

    // The integral is only advanced away from the clamp. At the clamp the outer
    // loop is asking for more heading than the maneuver allows, so winding on
    // would leave the loop stuck there after the offset reverses.
    float psiRef = -control::KX * x - control::KI * integral_;
    if (psiRef > control::PSI_REF_MAX) {
      psiRef = control::PSI_REF_MAX;
    } else if (psiRef < -control::PSI_REF_MAX) {
      psiRef = -control::PSI_REF_MAX;
    } else {
      integral_ += x * dt;
    }

    float delta = control::KP * (psiRef - psiF_) - control::KD * psiDot;
    if (delta > control::DELTA_MAX) delta = control::DELTA_MAX;
    if (delta < -control::DELTA_MAX) delta = -control::DELTA_MAX;

    float deg = servo::CENTRE_DEG + control::DIR * control::PER_WHEEL * delta *
                                        control::DEG_PER_RAD;
    if (deg < servo::MIN_DEG) deg = servo::MIN_DEG;
    if (deg > servo::MAX_DEG) deg = servo::MAX_DEG;

    Steer out;
    out.psiRef = psiRef;
    out.servoDeg = (int)lroundf(deg);
    return out;
  }

private:
  float integral_ = 0.0f;
  float psiF_ = 0.0f;
  float psiPrev_ = 0.0f;
  bool primed_ = false;

  // Centre with no heading demand, used by the deadband and by the switch.
  static Steer hold() {
    Steer out;
    out.psiRef = 0.0f;
    out.servoDeg = servo::CENTRE_DEG;
    return out;
  }
};
