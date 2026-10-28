#pragma once

#include <cmath>
#include <cstdio>

#include "ackermann.h"
#include "config.h"
#include "control.h"
#include "geometry.h"
#include "hal.h"

// The supervisor decides what the car is doing. The inner loop steers, and only one holds the servo at a time. BACK_IN reverses to the rear limit or a breach, CORRECT runs one timed stroke (heading first) and re-measures, DONE and GIVEUP brake. A translate stroke is an S-curve, and a rotate's drag is cleaned up next.

enum class Phase { IDLE, BACK_IN, CORRECT, DONE, GIVEUP };
enum class Stroke { NONE, ROTATE, TRANSLATE };

struct Telemetry {
  Phase phase = Phase::IDLE;
  Stroke stroke = Stroke::NONE;
  float t = 0;
  float x = 0;
  float psi = 0;
  float rear = 0;
  float gL = 0;
  float gR = 0;
  float servo = 90;
  float psiCap = 0;
  float clearance = 0;
  float peakYaw = 0;
  float travel = 0;
  int strokes = 0;
  bool valid = false;
  bool rearFault = false;
  bool travelFault = false;
  bool calibrated = false;
  bool outOfSpec = false;
};

struct Supervisor {
  // A copy would dangle, because ctl.c points into cfg and attach() injects the hal.
  Supervisor() = default;
  Supervisor(const Supervisor&) = delete;
  Supervisor& operator=(const Supervisor&) = delete;

  Config cfg;
  Hal* hal = nullptr;
  Control ctl;

  Phase phase = Phase::IDLE;
  Stroke stroke = Stroke::NONE;

  // Stroke sequencing, seconds and radians. sub walks one correction through 0 evaluate, 1 stroke running, 2 settling.
  int sub = 0;
  int strokeLeg = 0;
  float strokeT0 = 0;
  float strokeLegT0 = 0;
  float strokeDur = 0;
  float strokeDelta = 0;
  float strokeExcursion = 0;
  float strokeTargetPsi = 0;
  float strokeExcSign = 1.0f;
  float strokeStartX = 0;
  float strokeStartPsi = 0;
  float strokePlanned = 0;

  // Filtered estimate for the tolerance decisions.
  float filtX = 0.0f;
  float filtPsi = 0.0f;
  bool  filtPrimed = false;

  // Measured shift over planned shift, averaged over strokes big enough to mean something.
  float strokeGain = 1.0f;
  int strokeCount = 0;

  // A tolerance holds for cfg.dwell ticks before the supervisor acts, so the car does not chatter across the band.
  int dwellBad = 0;
  int inTolTicks = 0;

  // Exponential averages of the squared second difference of the heading and of the offset.
  float noisePsiAcc = 0.0f;
  float noiseXAcc = 0.0f;
  float lastRawPsi = 0.0f;
  float lastRawX = 0.0f;
  float prevRawPsi = 0.0f;
  float prevRawX = 0.0f;
  bool  noisePrimed = false;

  // One sample per tick while the car stands squared up, and the averages become S0 and the trim.
  int   calibCount = 0;
  float calibSumS = 0.0f;
  float calibSumT = 0.0f;

  int   rearFaultTicks = 0;
  float travel = 0.0f;
  int   invalidTicks = 0;

  // Last usable rear reading and the travel at that moment. Out of range is not a fault, and the distance reversed since tells the two apart.
  float rearSeen = 0.0f;
  float travelAtSeen = 0.0f;
  bool  rearSeenValid = false;

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

  // Park centred and squared up, then call this once. Averaging keeps one noisy reading out of the estimate for the rest of the run.
  void calibrate() {
    calibCount = 0;
    calibSumS = 0.0f;
    calibSumT = 0.0f;
    cfg.S0 = 0.0f;
    cfg.trim = 0.0f;
  }

  // Whether enough samples have been taken for the calibration to be good.
  bool calibrated() const { return calibCount >= cfg.calib_samples; }

  // Start reversing. The calibration has to be in place first, because S0 is what the heading estimate is measured against.
  bool start() {
    if (!calibrated()) return false;
    phase = Phase::BACK_IN;
    sub = 0;
    stroke = Stroke::NONE;
    strokeCount = 0;
    dwellBad = 0;
    peakYaw = 0.0f;
    travel = 0.0f;
    rearFaultTicks = 0;
    filtPrimed = false;
    noisePrimed = false;
    noisePsiAcc = 0.0f;
    noiseXAcc = 0.0f;
    inTolTicks = 0;
    invalidTicks = 0;
    rearSeenValid = false;
    rearSeen = 0.0f;
    travelAtSeen = 0.0f;
    last = -1.0f;
    tel.rearFault = false;
    tel.travelFault = false;
    tel.outOfSpec = false;
    ctl.reset();
    return true;
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

    // S0 is the average sensor sum with the car squared up, the trim the half difference.
    if (!calibrated() && geom::validRange(gL) && geom::validRange(gR)) {
      calibSumS += gL + gR;
      calibSumT += gL - gR;
      calibCount++;
      if (calibCount >= cfg.calib_samples) {
        const float n = (float)calibCount;
        cfg.S0 = calibSumS / n;
        cfg.trim = 0.5f * calibSumT / n;
      }
    }

    // No pose without S0, because the sensor sum would have no scale.
    geom::Pose p;
    if (calibrated()) p = geom::pose(cfg, gL, gR);

    // The raw estimate carries a couple of degrees of heading noise, so the tolerance decisions run on a filtered copy.
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

    // The noise is measured. The second difference of a smooth motion is sensor noise, where the first of a turning car is mostly yaw.
    if (p.ok && phase == Phase::BACK_IN) {
      if (noisePrimed) {
        const float d2Psi = p.psi - 2.0f * lastRawPsi + prevRawPsi;
        const float d2X = p.x - 2.0f * lastRawX + prevRawX;
        const float a = 0.05f;
        noisePsiAcc += a * (d2Psi * d2Psi - noisePsiAcc);
        noiseXAcc += a * (d2X * d2X - noiseXAcc);
      }
      prevRawPsi = lastRawPsi;
      prevRawX = lastRawX;
      lastRawPsi = p.psi;
      lastRawX = p.x;
      noisePrimed = true;
    }

    // Second difference carries six times the variance, and a first-order filter with coefficient a scales it by a/(2-a), which is what the tolerance covers.
    const float aFilt = 1.0f - expf(-cfg.dt / cfg.tau_filt);
    const float reduce = sqrtf(aFilt / (2.0f - aFilt));
    const float sigmaPsi = sqrtf(fabsf(noisePsiAcc) / 6.0f) * reduce;
    const float sigmaX = sqrtf(fabsf(noiseXAcc) / 6.0f) * reduce;
    float tolAlign = cfg.tol_align;
    float tolOff = cfg.tol_off;
    if (sigmaPsi * 3.0f > tolAlign) tolAlign = sigmaPsi * 3.0f;
    if (sigmaX * 3.0f > tolOff) tolOff = sigmaX * 3.0f;

    // The bay is made by hand, so its limits are recomputed every tick.
    const float clear = geom::clearance(cfg, gL, gR);
    float psiCap = geom::peakYawCap(cfg, clear);
    if (psiCap > cfg.psi_cap_max) psiCap = cfg.psi_cap_max;

    tel.t = now;
    tel.x = p.x;
    tel.psi = p.psi;
    tel.rear = gRear;
    tel.gL = gL;
    tel.gR = gR;
    tel.servo = servoDeg;
    tel.psiCap = psiCap;
    tel.clearance = clear;
    tel.peakYaw = peakYaw;
    tel.strokes = strokeCount;
    tel.travel = travel;
    if (p.ok && fabsf(p.psi) > peakYaw) peakYaw = fabsf(p.psi);
    tel.valid = p.ok;
    tel.calibrated = calibrated();

    switch (phase) {
      case Phase::IDLE:
        hal->setMotor(0);
        hal->setServo(90.0f);
        break;

      case Phase::BACK_IN:
        stepBackIn(dt, p, fp, gRear, psiCap, tolOff, tolAlign);
        break;

      case Phase::CORRECT:
        stepCorrect(now, dt, p, fp, gRear, psiCap, tolOff, tolAlign);
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
    // Reverse with the inner loop on the servo.
    hal->setMotor(-1, cfg.reverse_duty);
    travel += driveSpeed(cfg) * dt;

    if (p.ok) {
      const float delta = ctl.step(p.x, p.psi, -driveSpeed(cfg), dt, psiCap);
      servoDeg = 90.0f + delta * 57.29578f;
      hal->setServo(servoDeg);
    }

    // Rear limit. The trigger sits ahead of the wanted final gap by the assumed stopping distance and the latency margin.
    const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
    if (geom::validRange(gRear)) {
      rearSeen = gRear;
      travelAtSeen = travel;
      rearSeenValid = true;
      rearFaultTicks = 0;
      if (gRear <= trigger) {
        phase = Phase::DONE;
        stroke = Stroke::NONE;
        hal->setMotor(0);
        // Judge on the raw estimate, since the filtered copy lags by a filter time constant of travel.
        tel.outOfSpec = (fabsf(p.x) > cfg.tol_off) ||
                        (fabsf(p.psi) > cfg.tol_align);
        return;
      }
    } else if (rearSeenValid && rearSeen - (travel - travelAtSeen) > trigger) {
      // Out of range is not a fault, so keep reversing until the wall returns.
      rearFaultTicks = 0;
    } else if (++rearFaultTicks >= cfg.rear_fault_ticks) {
      // The wall should be in reach and the sensor cannot see it.
      phase = Phase::GIVEUP;
      stroke = Stroke::NONE;
      hal->setMotor(0);
      tel.rearFault = true;
      return;
    }

    // Travel watchdog, the backstop for a rear reading that is usable but wrong. With no odometer, the distance comes from the commanded speed.
    if (travel > cfg.back_in_max_travel) {
      phase = Phase::GIVEUP;
      stroke = Stroke::NONE;
      hal->setMotor(0);
      tel.travelFault = true;
      return;
    }

    // Out of tolerance needs a correction, but only after it has held for a few ticks, so drift does not chatter the car in and out of the band.
    if (fp.ok) {
      const bool bad = fabsf(fp.x) > tolOff || fabsf(fp.psi) > tolAlign;
      if (bad) {
        dwellBad++;
      } else {
        dwellBad = 0;
      }
      if (dwellBad >= cfg.dwell) {
        phase = Phase::CORRECT;
        sub = 0;
        stroke = Stroke::NONE;
        hal->setMotor(0);
        dwellBad = 0;
        ctl.reset();
      }
    }
  }

  // ======================================================= CORRECT ============
  // One correction. sub 0 evaluates and picks the next one, heading before offset, sub 1 runs the stroke, sub 2 settles before the next evaluation.
  void stepCorrect(float now, float dt, const geom::Pose& p,
                   const geom::Pose& fp, float gRear, float psiCap,
                   float tolOff, float tolAlign) {
    switch (sub) {
      case 0: {
        if (!p.ok) {
          // No usable measurement, so brake and wait rather than guess.
          hal->setMotor(0);
          return;
        }
        const bool headBad = fabsf(fp.psi) > tolAlign;
        const bool offBad = fabsf(fp.x) > tolOff;

        if (!headBad && !offBad) {
          // In tolerance. Hold it for a few ticks before reversing again, so the car does not chatter between the phases.
          if (++inTolTicks >= cfg.dwell) {
            phase = Phase::BACK_IN;
            sub = 0;
            stroke = Stroke::NONE;
            inTolTicks = 0;
            dwellBad = 0;
            ctl.reset();
          }
          return;
        }

        inTolTicks = 0;

        // Budget spent, so stop and flag it.
        if (strokeCount >= cfg.max_strokes) {
          phase = Phase::GIVEUP;
          stroke = Stroke::NONE;
          hal->setMotor(0);
          return;
        }

        // Heading first, because a standing error grows the offset and a translate stroke is only worth planning from a near-straight heading.
        if (headBad) {
          beginRotate(p, psiCap, now);
        } else {
          beginTranslate(p, psiCap, now);
        }
        break;
      }

      case 1: {
        const float t = now - strokeT0;
        float cmd;

        if (stroke == Stroke::ROTATE) {
          cmd = (t < strokeDur) ? strokeDelta : 0.0f;
        } else {
          // An S-curve takes one lock, then the opposite, switched on the measured excursion so it is speed independent.
          if (strokeLeg == 0) {
            cmd = strokeDelta;
          } else {
            cmd = -strokeDelta;
          }
          if (t >= strokeDur) cmd = 0.0f;
        }

        servoDeg = 90.0f + cmd * 57.29578f;
        hal->setServo(servoDeg);
        const int strokeDir = (cfg.stroke_dir > 0) ? 1 : -1;
        hal->setMotor(strokeDir, cfg.stroke_duty);

        bool done = (t >= strokeDur);
        bool abort = false;

        // A stroke driving towards the wall is watched like the reverse. Strokes normally run forward, so this only bites if the config turns them.
        bool rearHit = false;
        if (strokeDir < 0) {
          travel += driveSpeed(cfg) * dt;
          const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
          if (geom::validRange(gRear) && gRear <= trigger) rearHit = true;
        }

        // Losing the side readings stops the stroke, because the car may be against a wall and holding the lock makes that worse.
        if (p.ok) {
          invalidTicks = 0;
        } else if (++invalidTicks >= cfg.stroke_invalid_ticks) {
          abort = true;
        }

        if (p.ok) {
          const float excursion = p.psi - strokeStartPsi;

          if (stroke == Stroke::ROTATE) {
            // Closed loop on the heading, stopping when the car has turned by the amount asked for.
            const float change = p.psi - strokeStartPsi;
            if (strokeTargetPsi > 0.0f ? change >= strokeTargetPsi - 0.010f
                                       : change <= strokeTargetPsi + 0.010f) {
              done = true;
            }
          } else {
            if (strokeLeg == 0) {
              // The first leg runs to the planned excursion.
              if (strokeExcSign * excursion >= strokeExcursion) {
                strokeLeg = 1;
                strokeLegT0 = now;
              }
            } else {
              // The second leg steers back a little early, because the servo takes time to centre. The 0.7 corrects a 2.4x slew overestimate.
              if (strokeExcSign * excursion <=
                  0.7f * slewOvershoot(fabsf(strokeDelta)))
                done = true;
            }
          }

        }

        // Clearance abort on the filtered heading, so the interlock still works when the measurement does not.
        if (fabsf(fp.psi) > psiCap + 0.087f) abort = true;

        if (abort || rearHit || t > cfg.stroke_timeout) done = true;

        if (done) {
          hal->setMotor(0);
          hal->setServo(90.0f);
          servoDeg = 90.0f;

          // Learn the shift produced against the one planned, which absorbs the model, the surface, the speed and the servo.
          if (stroke == Stroke::TRANSLATE && p.ok && strokePlanned > 0.015f) {
            const float achieved = fabsf(p.x - strokeStartX);
            float g = achieved / strokePlanned;
            if (g < 0.4f) g = 0.4f;
            if (g > 2.5f) g = 2.5f;
            strokeGain = 0.5f * strokeGain + 0.5f * g;
          }

          if (trace) {
            printf("    stroke end: x=%.1fmm psi=%.2fdeg (was x=%.1fmm "
                   "psi=%.2fdeg) dur=%.2fs gain=%.2f\n",
                   p.ok ? p.x * 1000.0f : 0.0f,
                   p.ok ? p.psi * 57.29578f : 0.0f,
                   strokeStartX * 1000.0f, strokeStartPsi * 57.29578f,
                   now - strokeT0, strokeGain);
          }
          stroke = Stroke::NONE;
          if (rearHit) {
            // The rear limit during a stroke ends the maneuver, as in reverse.
            phase = Phase::DONE;
            sub = 0;
            tel.outOfSpec = (fabsf(p.x) > cfg.tol_off) ||
                            (fabsf(p.psi) > cfg.tol_align);
          } else {
            sub = 2;
          }
        }
        break;
      }

      case 2: {
        // Settle with the brakes on and the wheels centred, and wait for the car to be still.
        hal->setMotor(0);
        hal->setServo(90.0f);
        servoDeg = 90.0f;
        if (now - strokeT0 >= strokeDur + cfg.brake_settle) {
          strokeCount++;
          sub = 0;
        }
        break;
      }
    }
  }

  // =============================================== stroke planning ============
  // Exact and speed independent. dx/dpsi = -(L/tan delta) * sin(psi) holds at every instant, and a stroke stops on the measured excursion, not a timer. translate shifts |dx| = 2 * L * (1 - cos(psi_peak)) / tan(delta) and rotate drags |drag| = L * (1 - cos(dpsi)) / tan(delta). Deadband and slew are modelled, and a per-stroke gain absorbs the rest.

  // Largest equivalent angle the steering can deliver, which under Ackermann is the inner wheel's stop.
  float steerLimit() {
    if (cfg.steer_model == SteerModel::ACKERMANN) return ack::deltaEqMax(cfg);
    return cfg.delta_max;
  }

  // Stroke amplitude for an offset, limited by what the steering can deliver.
  float strokeAmp(float offset) {
    float a = (fabsf(offset) <= cfg.fine_threshold) ? cfg.delta_fine
                                                    : cfg.delta_coarse;
    const float lim = steerLimit();
    if (a > lim) a = lim;
    return a;
  }

  // The steer angle the wheels actually reach for a given command.
  float effAmp(float amp) {
    float a = amp - cfg.servo_deadband;
    if (a < 0.05f) a = 0.05f;
    return a;
  }

  // Rough time for a heading excursion, used only as a safety limit.
  float excursionTime(float dpsi, float amp) {
    const float tanD = tanf(effAmp(amp));
    const float v = driveSpeed(cfg);
    if (tanD < 1e-6f || v < 1e-3f) return cfg.stroke_timeout;
    return fabsf(dpsi) * car::L_wb / (v * tanD);
  }

  // How far past the trigger the peak lands while the servo swings to the opposite lock.
  float slewOvershoot(float amp) {
    const float tanD = tanf(effAmp(amp));
    const float yawRate = cfg.speed * tanD / car::L_wb;
    return yawRate * (effAmp(amp) / cfg.servo_rate);
  }

  void beginRotate(const geom::Pose& p, float psiCap, float now) {
    // Null the heading in steps the measurement can resolve, capped by the clearance.
    float limit = psiCap;
    if (limit > cfg.psi_meas_max) limit = cfg.psi_meas_max;

    // The heading change asked for, towards zero and no more than the cap allows in one step.
    float want = -p.psi;
    if (want > limit) want = limit;
    if (want < -limit) want = -limit;

    const float amp = strokeAmp(p.psi);

    // sign(dpsi) = -sign(cmd) * dir.
    const float dir = (cfg.stroke_dir > 0) ? 1.0f : -1.0f;
    const float delta = -((want >= 0.0f) ? 1.0f : -1.0f) * dir * amp;

    strokeDur = excursionTime(want, amp) * 1.8f;
    if (strokeDur > cfg.stroke_timeout) strokeDur = cfg.stroke_timeout;

    stroke = Stroke::ROTATE;
    invalidTicks = 0;
    strokeDelta = delta;
    strokeStartX = p.x;
    strokeStartPsi = p.psi;
    strokeTargetPsi = want;
    strokeExcursion = 0.0f;
    strokeExcSign = (want >= 0.0f) ? 1.0f : -1.0f;
    strokeLeg = 0;
    strokeT0 = now;
    sub = 1;
    if (trace) {
      printf("    ROTATE  want dpsi=%.2fdeg amp=%.1fdeg limit=%.2fs "
             "(x=%.1fmm psi=%.2fdeg)\n",
             want * 57.29578f, amp * 57.29578f, strokeDur, p.x * 1000.0f,
             p.psi * 57.29578f);
    }
  }

  void beginTranslate(const geom::Pose& p, float psiCap, float now) {
    const float amp = strokeAmp(p.x);
    const float dir = (cfg.stroke_dir > 0) ? 1.0f : -1.0f;

    // The first leg swings towards the centreline, so the command sign and the excursion direction are fixed.
    const float first = (p.x > 0.0f) ? amp : -amp;
    const float sgnFirst = (first > 0.0f) ? 1.0f : -1.0f;
    const float excSign = -sgnFirst * dir;

    // The tangent carries the sign of the first leg. An unsigned one plans strokes that push a car past the centreline further away.
    const float tanD = sgnFirst * tanf(effAmp(amp));

    // A symmetric S-curve returns the heading and shifts 2(L/tan delta)(cos psi_peak - cos psi_start), so the plan works in the peak, not the excursion.
    const float cs = cosf(p.psi);

    // Capping the peak caps the shift and stays inside the cone-safe window, where the 1/cos bend is worth a tenth of a degree.
    float cap = psiCap;
    if (cap > cfg.psi_meas_max) cap = cfg.psi_meas_max;

    // Room left in that window. Swinging away from the centreline runs out of it, swinging through it has the far side.
    const bool away = (excSign * p.psi >= 0.0f);
    float room = away ? (cap - fabsf(p.psi)) : (cap + fabsf(p.psi));
    if (room < 0.0f) room = 0.0f;

    // Largest shift available, peak at the cap, signed by the tangent.
    float dxMax = 2.0f * (car::L_wb / tanD) * (cosf(cap) - cs) * strokeGain;
    if (away && room <= 0.0f) dxMax = 0.0f;

    // The shift asked for, towards the centreline, limited by one stroke.
    const float dxWant = -p.x;
    float dx = dxWant;
    if (fabsf(dx) > fabsf(dxMax)) dx = dxMax;

    // The peak that produces it. The gain is what previous strokes achieved against the model, so the model is asked for a larger excursion.
    float c = cs + (dx / strokeGain) * tanD / (2.0f * car::L_wb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    const float peakMag = acosf(c);

    // Two candidate peaks, +peakMag and -peakMag, and the excursion has to go in the forced direction.
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

    // The peak lands past the trigger while the servo swings, so ask for less.
    float target = fabsf(exc) - slewOvershoot(amp);
    if (target < 0.02f) target = 0.02f;

    strokeDur = excursionTime(target, amp) * 2.0f * 1.8f;
    if (strokeDur > cfg.stroke_timeout) strokeDur = cfg.stroke_timeout;

    stroke = Stroke::TRANSLATE;
    invalidTicks = 0;
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
    // Ramp the servo back to centre rather than snapping it.
    const float step = 180.0f * dt;
    if (servoDeg > 90.0f + step) servoDeg -= step;
    else if (servoDeg < 90.0f - step) servoDeg += step;
    else servoDeg = 90.0f;
    hal->setServo(servoDeg);
  }
};
