#pragma once

#include <cmath>

#include "config.h"
#include "control.h"
#include "geometry.h"
#include "hal.h"

// The supervisor decides what the car is doing. The inner loop decides the
// steering. They never fight over the servo: during a correction stroke the
// supervisor has it, otherwise the inner loop does.
//
//   BACK_IN   reverse, with the inner loop on the servo, until the rear limit
//             is reached or the car drifts out of tolerance
//   CORRECT   one correction at a time, heading before offset. Each is a
//             timed stroke with the motor running and the servo following a
//             profile. The car is re-measured after every stroke.
//   DONE      motor off, brake on, steering centred
//   GIVEUP    brake and flag it, once the stroke budget is spent
//
// A stroke is not one held steering angle. A translate stroke is an S-curve:
// the servo holds one lock for the first half and the opposite lock for the
// second half, so the heading excursion cancels itself and the car ends up
// moved sideways with its heading where it started. A rotate stroke holds one
// lock for its whole duration. That leaves a lateral drag behind, and the
// drag is what the next translate stroke cleans up.
//
// The stroke amplitudes and durations come from PLAN.md section 6.3:
//
//   translate:  dx = L_wb * psi_peak^2 / tan(delta)
//               T   = 2 * sqrt(dx * L_wb / (v^2 * tan(delta)))
//   rotate:     dpsi = v * tan(delta) / L_wb * T

enum class Phase { IDLE, BACK_IN, CORRECT, DONE, GIVEUP };
enum class Stroke { NONE, ROTATE, TRANSLATE };

struct Telemetry {
  Phase phase = Phase::IDLE;
  Stroke stroke = Stroke::NONE;
  float t = 0;
  float x = 0;
  float psi = 0;
  float rear = 0;
  float servo = 90;
  float psiCap = 0;
  float clearance = 0;
  float peakYaw = 0;
  int strokes = 0;
  bool valid = false;
};

struct Supervisor {
  Config cfg;
  Hal* hal = nullptr;
  Control ctl;

  Phase phase = Phase::IDLE;
  Stroke stroke = Stroke::NONE;

  // Stroke sequencing. Times are seconds, angles radians, distances metres.
  // `sub` walks the three stages of one correction: 0 evaluate, 1 stroke
  // running, 2 settling. `strokeLeg` picks a leg of an S-curve, 0 first or
  // 1 second. `strokeExcursion` is the heading excursion being aimed for and
  // `strokeTargetPsi` the signed heading change being asked for, with
  // `strokeExcSign` carrying that sign. ---
  int sub = 0;
  int strokeLeg = 0;
  float strokeT0 = 0;
  float strokeLegT0 = 0;
  float strokeDur = 0;
  float strokeHalf = 0;
  float strokeDelta = 0;
  float strokeExcursion = 0;
  float strokeTargetPsi = 0;
  float strokeExcSign = 1.0f;
  float strokeStartX = 0;
  float strokeStartPsi = 0;
  float strokePlanned = 0;
  // end

  // Filtered copies of the estimate, used for the tolerance decisions.
  // `filtPrimed` says whether the filters have seen a sample yet. ---
  float filtX = 0.0f;
  float filtPsi = 0.0f;
  bool  filtPrimed = false;
  // end

  // The measured shift over the planned shift. It is averaged over strokes
  // that were big enough for the measurement to mean anything. ---
  float strokeGain = 1.0f;
  int strokeCount = 0;
  // end

  // Dwell counters. A tolerance has to hold for `dwell` ticks before the
  // supervisor acts on it, so the car does not chatter across the band. ---
  int dwellOk = 0;
  int dwellBad = 0;
  int inTolTicks = 0;
  // end

  // Noise measurement. The two accumulators are exponential averages of the
  // squared difference between consecutive estimates of the heading and of
  // the offset. ---
  float noisePsiAcc = 0.0f;
  float noiseXAcc = 0.0f;
  float lastRawPsi = 0.0f;
  float lastRawX = 0.0f;
  bool  noisePrimed = false;
  // end

  float last = -1.0f;
  float servoDeg = 90.0f;
  float peakYaw = 0.0f;
  bool trace = false;
  Telemetry tel;

  void attach(Hal* h) {
    hal = h;
    ctl.c = &cfg;
    ctl.reset();
  }

  // Park the car centred and squared up, then call this once. S0 folds the
  // bay width and both lateral mount offsets into one number, and the trim
  // takes the left/right mount asymmetry out of the offset.
  bool calibrate() {
    if (hal == nullptr) return false;
    float gL, gR, gRear;
    hal->readSensors(gL, gR, gRear);
    if (!geom::validRange(gL) || !geom::validRange(gR)) return false;
    cfg.S0 = gL + gR;
    cfg.trim = 0.5f * (gL - gR);
    return true;
  }

  void start() {
    phase = Phase::BACK_IN;
    sub = 0;
    stroke = Stroke::NONE;
    strokeCount = 0;
    dwellOk = dwellBad = 0;
    peakYaw = 0.0f;
    ctl.reset();
  }

  void stop() {
    phase = Phase::DONE;
    sub = 0;
    stroke = Stroke::NONE;
    if (hal) {
      hal->setMotor(0);
      hal->setServo(90.0f);
    }
  }

  // One supervisor tick, called at the sensor rate.
  void update() {
    if (hal == nullptr) return;

    const float now = hal->now();
    float dt = (last < 0.0f) ? cfg.dt : (now - last);
    if (dt <= 0.0f) dt = cfg.dt;
    if (dt > 0.5f) dt = 0.5f;
    last = now;

    float gL, gR, gRear;
    hal->readSensors(gL, gR, gRear);
    const geom::Pose p = geom::pose(cfg, gL, gR);

    // The raw estimate carries a couple of degrees of heading noise, so the
    // tolerance decisions run on a filtered copy. The control law uses the raw
    // estimate and filters it itself. ---
    if (p.ok) {
      if (!filtPrimed) {
        filtX = p.x;
        filtPsi = p.psi;
        filtPrimed = true;
      } else {
        const float a = 1.0f - expf(-dt / cfg.tau_filt);
        filtX += a * (p.x - filtX);
        filtPsi += a * (p.psi - filtPsi);
      }
    }
    const geom::Pose fp{filtX, filtPsi, p.ok};
    // end

    // The tolerances have to be wider than the noise the sensors actually
    // produce, and that is not known until the hardware runs. So it gets
    // measured: the spread of the estimate between samples is noise while the
    // car is tracking, and the tolerance follows it. ---
    if (p.ok && phase == Phase::BACK_IN) {
      if (noisePrimed) {
        const float dPsi = p.psi - lastRawPsi;
        const float dX = p.x - lastRawX;
        const float a = 0.05f;
        noisePsiAcc += a * (dPsi * dPsi - noisePsiAcc);
        noiseXAcc += a * (dX * dX - noiseXAcc);
      }
      lastRawPsi = p.psi;
      lastRawX = p.x;
      noisePrimed = true;
    }
    // end

    // The variance of a difference between two independent samples is twice
    // the variance, and an exponential average of that gives the variance of
    // the raw estimate. The filter then reduces it by a known factor. ---
    const float reduce = sqrtf(cfg.tau_filt / (2.0f * cfg.dt));
    const float sigmaPsi = sqrtf(fabsf(noisePsiAcc) * 0.5f) * reduce;
    const float sigmaX = sqrtf(fabsf(noiseXAcc) * 0.5f) * reduce;
    float tolAlign = cfg.tol_align;
    float tolOff = cfg.tol_off;
    if (sigmaPsi * 3.0f > tolAlign) tolAlign = sigmaPsi * 3.0f;
    if (sigmaX * 3.0f > tolOff) tolOff = sigmaX * 3.0f;
    // end

    // Runtime limits, recomputed every tick from the live geometry. The bay is
    // made by hand, so none of this is a design-time constant. ---
    const float clear = geom::clearance(cfg, gL, gR);
    float psiCap = geom::peakYawCap(cfg, clear);
    if (psiCap > cfg.psi_cap_max) psiCap = cfg.psi_cap_max;
    // end

    tel.t = now;
    tel.x = p.x;
    tel.psi = p.psi;
    tel.rear = gRear;
    tel.servo = servoDeg;
    tel.psiCap = psiCap;
    tel.clearance = clear;
    tel.peakYaw = peakYaw;
    tel.strokes = strokeCount;
    tel.valid = p.ok;

    switch (phase) {
      case Phase::IDLE:
        hal->setMotor(0);
        hal->setServo(90.0f);
        break;

      case Phase::BACK_IN:
        stepBackIn(dt, p, fp, gRear, psiCap, tolOff, tolAlign);
        break;

      case Phase::CORRECT:
        stepCorrect(now, p, fp, psiCap, tolOff, tolAlign);
        break;

      case Phase::DONE:
      case Phase::GIVEUP:
        hal->setMotor(0);
        centreSteering(dt);
        break;
    }

    tel.phase = phase;
    tel.stroke = stroke;
    tel.servo = servoDeg;
  }

  // ======================================================= BACK_IN ============
  void stepBackIn(float dt, const geom::Pose& p, const geom::Pose& fp,
                  float gRear, float psiCap, float tolOff, float tolAlign) {
    // Reverse, with the inner loop on the servo. ---
    hal->setMotor(-1);

    if (p.ok) {
      const float delta = ctl.step(p.x, p.psi, -cfg.speed, dt, psiCap);
      servoDeg = 90.0f + delta * 57.29578f;
      hal->setServo(servoDeg);
    }
    // end

    // Rear limit. The trigger sits in front of the wanted final gap by the
    // measured stopping distance plus the sensor latency margin. ---
    const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
    if (geom::validRange(gRear) && gRear <= trigger) {
      phase = Phase::DONE;
      stroke = Stroke::NONE;
      hal->setMotor(0);
      return;
    }
    // end

    // Out of tolerance, so a correction is needed. That only happens once the
    // condition has held for a few ticks, so the car does not chatter in and
    // out of the band while it drifts. ---
    if (fp.ok) {
      const bool bad = fabsf(fp.x) > tolOff || fabsf(fp.psi) > tolAlign;
      if (bad) {
        dwellBad++;
        dwellOk = 0;
      } else {
        dwellBad = 0;
        dwellOk++;
      }
      if (dwellBad >= cfg.dwell) {
        phase = Phase::CORRECT;
        sub = 0;
        stroke = Stroke::NONE;
        hal->setMotor(0);
        ctl.reset();
      }
    }
    // end
  }

  // ======================================================= CORRECT ============
  // Run one correction. Sub-state 0 evaluates and picks the next
  // correction, heading before offset. Sub-state 1 runs the stroke. Sub-state
  // 2 settles the car before the next evaluation.
  void stepCorrect(float now, const geom::Pose& p, const geom::Pose& fp,
                   float psiCap, float tolOff, float tolAlign) {
    switch (sub) {
      case 0: {
        if (!p.ok) {
          // No usable measurement. Brake and wait rather than guess. ---
          hal->setMotor(0);
          return;
          // end
        }
        const bool headBad = fabsf(fp.psi) > tolAlign;
        const bool offBad = fabsf(fp.x) > tolOff;

        if (!headBad && !offBad) {
          // In tolerance. Hold it for a few ticks before going back to
          // reversing, so the car does not chatter between the phases. ---
          if (++inTolTicks >= cfg.dwell) {
            phase = Phase::BACK_IN;
            sub = 0;
            stroke = Stroke::NONE;
            inTolTicks = 0;
            ctl.reset();
          }
          return;
          // end
        }

        inTolTicks = 0;

        // Still out of tolerance. If the budget is spent, stop and flag it. ---
        if (strokeCount >= cfg.max_strokes) {
          phase = Phase::GIVEUP;
          stroke = Stroke::NONE;
          hal->setMotor(0);
          return;
        }
        // end

        // Heading first. A standing heading error makes the offset grow, and
        // a translate stroke is only worth planning from a heading that is
        // already close to straight. ---
        if (headBad) {
          beginRotate(p, psiCap, now);
        } else {
          beginTranslate(p, psiCap, now);
        }
        // end
        break;
      }

      case 1: {
        const float t = now - strokeT0;
        float cmd;

        if (stroke == Stroke::ROTATE) {
          cmd = (t < strokeDur) ? strokeDelta : 0.0f;
        } else {
          // S-curve: one lock for the first leg, the opposite for the second.
          // The switch between the legs is triggered by the measured heading
          // excursion, not by a timer, so the excursion is the same whatever
          // the speed does. ---
          if (strokeLeg == 0) {
            cmd = strokeDelta;
          } else {
            cmd = -strokeDelta;
          }
          if (t >= strokeDur) cmd = 0.0f;
          // end
        }

        servoDeg = 90.0f + cmd * 57.29578f;
        hal->setServo(servoDeg);
        hal->setMotor(cfg.stroke_dir > 0 ? 1 : -1);

        bool done = (t >= strokeDur);
        bool abort = false;

        if (p.ok) {
          const float excursion = p.psi - strokeStartPsi;

          if (stroke == Stroke::ROTATE) {
            // Closed loop on the heading change: stop when the car has
            // rotated by the amount that was asked for. ---
            const float change = p.psi - strokeStartPsi;
            if (strokeTargetPsi > 0.0f ? change >= strokeTargetPsi - 0.010f
                                       : change <= strokeTargetPsi + 0.010f) {
              done = true;
            }
            // end
          } else {
            if (strokeLeg == 0) {
              // First leg: run until the planned excursion is reached. ---
              if (strokeExcSign * excursion >= strokeExcursion) {
                strokeLeg = 1;
                strokeLegT0 = now;
              }
              // end
            } else {
              // Second leg: steer back until the heading is where it
              // started. The band is zero width on purpose, because the
              // estimate is noisier than any band worth having. ---
              if (strokeExcSign * excursion <= 0.0f) done = true;
              // end
            }
          }

          // Clearance abort: never let the body swing past what the bay
          // holds. ---
          if (fabsf(p.psi) > psiCap + 0.087f) abort = true;
          // end
        }

        if (abort || t > cfg.stroke_timeout) done = true;

        if (done) {
          hal->setMotor(0);
          hal->setServo(90.0f);
          servoDeg = 90.0f;

          // Learn from the result: the shift a stroke actually produced
          // against the shift the plan asked for. This absorbs whatever the
          // model gets wrong, plus the surface, the speed and the servo. ---
          if (stroke == Stroke::TRANSLATE && p.ok && strokePlanned > 0.015f) {
            const float achieved = fabsf(p.x - strokeStartX);
            float g = achieved / strokePlanned;
            if (g < 0.4f) g = 0.4f;
            if (g > 2.5f) g = 2.5f;
            strokeGain = 0.5f * strokeGain + 0.5f * g;
          }
          // end

          if (trace) {
            printf("    stroke end: x=%.1fmm psi=%.2fdeg (was x=%.1fmm "
                   "psi=%.2fdeg) dur=%.2fs gain=%.2f\n",
                   p.ok ? p.x * 1000.0f : 0.0f,
                   p.ok ? p.psi * 57.29578f : 0.0f,
                   strokeStartX * 1000.0f, strokeStartPsi * 57.29578f,
                   now - strokeT0, strokeGain);
          }
          stroke = Stroke::NONE;
          sub = 2;
        }
        break;
      }

      case 2: {
        // Settle: brakes on, wheels centred, wait for the car to be still. ---
        hal->setMotor(0);
        hal->setServo(90.0f);
        servoDeg = 90.0f;
        if (now - strokeT0 >= strokeDur + cfg.brake_settle) {
          strokeCount++;
          sub = 0;
        }
        // end
        break;
      }
    }
  }

  // =============================================== stroke planning ============
  // Both stroke relations are exact and independent of speed and ramp. They
  // come from dx/dpsi = -(L/tan delta) * sin(psi), which holds at every
  // instant whatever the speed happens to be, so a stroke lands where it is
  // aimed even if the battery sags.
  //
  //   translate S-curve:  |dx|   = 2 * L * (1 - cos(psi_peak)) / tan(delta)
  //   rotate drag:        |drag| =     L * (1 - cos(dpsi))    / tan(delta)
  //
  // The peak excursion of a translate stroke sets the displacement, and the
  // stroke is stopped on the measured excursion rather than on a timer, which
  // is what makes it speed independent.
  //
  // Two things stand between the plan and the floor. The servo deadband means
  // the wheel angle is the command less the deadband, and the effective angle
  // is what the relations need. The servo slew means the car keeps yawing for
  // a moment after the command flips to the opposite lock, so the peak
  // overshoots the trigger. Both are modelled, and whatever is left over is
  // absorbed by a gain the supervisor learns from the measured result of each
  // stroke.

  // The steer angle the wheels actually reach for a given command.
  float effAmp(float amp) {
    float a = amp - cfg.servo_deadband;
    if (a < 0.05f) a = 0.05f;
    return a;
  }

  // Heading excursion needed to shift sideways by dx.
  float translateExcursion(float dx, float amp) {
    const float tanD = tanf(effAmp(amp));
    float c = 1.0f - fabsf(dx) * tanD / (2.0f * cfg.L_wb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c);
  }

  // Lateral shift a given peak excursion produces.
  float translateShift(float exc, float amp) {
    const float tanD = tanf(effAmp(amp));
    return 2.0f * cfg.L_wb * (1.0f - cosf(exc)) / tanD;
  }

  // Rough time for a heading excursion, used only as a safety limit.
  float excursionTime(float dpsi, float amp) {
    const float tanD = tanf(effAmp(amp));
    if (tanD < 1e-6f || cfg.speed < 1e-3f) return cfg.stroke_timeout;
    return fabsf(dpsi) * cfg.L_wb / (cfg.speed * tanD);
  }

  // How far past the trigger the peak excursion lands, because the servo
  // needs time to swing to the opposite lock.
  float slewOvershoot(float amp) {
    const float tanD = tanf(effAmp(amp));
    const float yawRate = cfg.speed * tanD / cfg.L_wb;
    return yawRate * (effAmp(amp) / cfg.servo_rate);
  }

  void beginRotate(const geom::Pose& p, float psiCap, float now) {
    // Aim to null the heading. The rotation is split into steps the
    // measurement can resolve, and the peak yaw is capped by the clearance.
    // ---
    float limit = psiCap;
    if (limit > cfg.psi_meas_max) limit = cfg.psi_meas_max;
    // end

    // The heading change being asked for: towards zero, and no more than the
    // cap allows in one step. ---
    float want = -p.psi;
    if (want > limit) want = limit;
    if (want < -limit) want = -limit;
    // end

    const float amp = (fabsf(p.psi) <= cfg.fine_threshold) ? cfg.delta_fine
                                                            : cfg.delta_coarse;

    // The heading change follows the sign of the steering angle and the
    // direction of travel: sign(dpsi) = -sign(cmd) * dir. ---
    const float dir = (cfg.stroke_dir > 0) ? 1.0f : -1.0f;
    const float delta = -((want >= 0.0f) ? 1.0f : -1.0f) * dir * amp;
    // end

    strokeDur = excursionTime(want, amp) * 1.8f;
    if (strokeDur > cfg.stroke_timeout) strokeDur = cfg.stroke_timeout;

    stroke = Stroke::ROTATE;
    strokeDelta = delta;
    strokeStartX = p.x;
    strokeStartPsi = p.psi;
    strokeTargetPsi = want;
    strokeExcursion = 0.0f;
    strokeExcSign = (want >= 0.0f) ? 1.0f : -1.0f;
    strokeLeg = 0;
    strokeT0 = now;
    strokeHalf = strokeDur * 0.5f;
    sub = 1;
    if (trace) {
      printf("    ROTATE  want dpsi=%.2fdeg amp=%.1fdeg limit=%.2fs "
             "(x=%.1fmm psi=%.2fdeg)\n",
             want * 57.29578f, amp * 57.29578f, strokeDur, p.x * 1000.0f,
             p.psi * 57.29578f);
    }
  }

  void beginTranslate(const geom::Pose& p, float psiCap, float now) {
    const float amp = (fabsf(p.x) <= cfg.fine_threshold) ? cfg.delta_fine
                                                          : cfg.delta_coarse;
    const float tanD = tanf(effAmp(amp));
    const float dir = (cfg.stroke_dir > 0) ? 1.0f : -1.0f;

    // The first leg always swings the heading in the direction that moves the
    // car towards the centreline, so the sign of the first leg's command is
    // fixed, and so is the direction the excursion goes in. The heading change
    // follows sign(dpsi) = -sign(cmd) * dir. ---
    const float first = (p.x > 0.0f) ? amp : -amp;
    const float sgnFirst = (first > 0.0f) ? 1.0f : -1.0f;
    const float excSign = -sgnFirst * dir;
    // end

    // The shift of a symmetric S-curve that returns the heading to where it
    // started is
    //   dx = 2 * (L / tan(delta)) * (cos(psi_peak) - cos(psi_start))
    // so the plan works in the peak heading, not in the excursion. That
    // matters: an excursion measured from a start that is already yawed can
    // land the peak near zero, where the shift is almost insensitive to it.
    // ---
    const float cs = cosf(p.psi);
    // end

    // The peak is capped, which caps the shift. The cap also keeps the
    // absolute heading inside the cone-safe window, so the measurement stays
    // valid for the whole stroke. ---
    float cap = psiCap;
    if (cap > cfg.psi_meas_max) cap = cfg.psi_meas_max;
    // end

    // The largest shift available: put the peak at the cap. It only ever
    // shifts towards the centreline, so it is negative, and dxMax is the cap
    // on how far the car may move in one stroke. ---
    float dxMax = 2.0f * (cfg.L_wb / tanD) * (cosf(cap) - cs) * strokeGain;
    if (dxMax > 0.0f) dxMax = 0.0f;

    float dx = -p.x;
    if (dx < dxMax) dx = dxMax;
    if (dx > 0.0f) dx = 0.0f;
    // end

    // The peak that produces that shift. The gain is what previous strokes
    // actually achieved against what the model predicted, so the model is
    // asked for a proportionally larger excursion to land on the target. ---
    float c = cs + (dx / strokeGain) * tanD / (2.0f * cfg.L_wb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    const float peakMag = acosf(c);
    // end

    // Two candidate peaks, and the excursion has to go in the forced
    // direction. The first candidate peaks at +peakMag, the second at
    // -peakMag. ---
    const float excA = peakMag - p.psi;
    const float excB = -peakMag - p.psi;
    float exc;
    if (excSign * excA > 0.0f) {
      exc = excA;
    } else if (excSign * excB > 0.0f) {
      exc = excB;
    } else {
      // Neither works, so take the smaller swing.
      exc = (fabsf(excA) <= fabsf(excB)) ? excA : excB;
    }
    // end

    // The peak lands past the trigger, because the servo needs time to swing
    // to the opposite lock. Ask for a smaller excursion to land on target. ---
    float target = fabsf(exc) - slewOvershoot(amp);
    if (target < 0.02f) target = 0.02f;
    // end

    strokeDur = excursionTime(target, amp) * 2.0f * 1.8f;
    if (strokeDur > cfg.stroke_timeout) strokeDur = cfg.stroke_timeout;
    strokeHalf = strokeDur * 0.5f;

    stroke = Stroke::TRANSLATE;
    strokeDelta = first;
    strokeStartX = p.x;
    strokeStartPsi = p.psi;
    strokeExcursion = target;
    strokeExcSign = (exc >= 0.0f) ? 1.0f : -1.0f;
    strokeLeg = 0;
    strokeT0 = now;
    strokePlanned = fabsf(dx);
    sub = 1;
    if (trace) {
      printf("    TRANSLATE want dx=%.1fmm amp=%.1fdeg peak=%.1fdeg "
             "exc=%.1fdeg limit=%.2fs (x=%.1fmm psi=%.2fdeg) dxMax=%.1fmm "
             "gain=%.2f\n",
             dx * 1000.0f, amp * 57.29578f, peakMag * 57.29578f,
             target * 57.29578f, strokeDur, p.x * 1000.0f,
             p.psi * 57.29578f, dxMax * 1000.0f, strokeGain);
    }
  }

  void centreSteering(float dt) {
    // Ramp the servo back to centre rather than snapping it. ---
    const float step = 180.0f * dt;
    if (servoDeg > 90.0f + step) servoDeg -= step;
    else if (servoDeg < 90.0f - step) servoDeg += step;
    else servoDeg = 90.0f;
    hal->setServo(servoDeg);
    // end
  }
};
