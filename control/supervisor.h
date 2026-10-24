#pragma once

#include <cmath>
#include <cstdio>

#include "ackermann.h"
#include "config.h"
#include "control.h"
#include "geometry.h"
#include "hal.h"

// the supervisor decides what the car is doing. the inner loop decides the
// steering. they never fight over the servo: during a correction stroke the
// supervisor has it, otherwise the inner loop does.
//
//   BACK_IN   reverse, with the inner loop on the servo, until the rear limit
//             is reached or the car drifts out of tolerance
//   CORRECT   one correction at a time, heading before offset. each is a timed
//             stroke with the motor running and the servo following a profile.
//             the car is re-measured after every stroke.
//   DONE      motor off, brake on, steering centred
//   GIVEUP    brake and flag it, once the stroke budget is spent
//
// a stroke is not one held steering angle. a translate stroke is an S-curve:
// the servo holds one lock for the first half and the opposite lock for the
// second half, so the heading excursion cancels itself and the car ends up
// moved sideways with its heading where it started. a rotate stroke holds one
// lock for its whole duration. that leaves a lateral drag behind, and the drag
// is what the next translate stroke cleans up.
//
// the stroke amplitudes and durations come from PLAN.md section 6.3:
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
  float travel = 0;
  int strokes = 0;
  bool valid = false;
  bool rearFault = false;
  bool travelFault = false;
  bool calibrated = false;
  bool outOfSpec = false;
};

struct Supervisor {
  // a copy would dangle: ctl.c points into this struct's own cfg and attach()
  // injects the hal. ---
  Supervisor() = default;
  Supervisor(const Supervisor&) = delete;
  Supervisor& operator=(const Supervisor&) = delete;
  // end

  Config cfg;
  Hal* hal = nullptr;
  Control ctl;

  Phase phase = Phase::IDLE;
  Stroke stroke = Stroke::NONE;

  // stroke sequencing. times are seconds, angles radians, distances metres.
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
  float strokeDelta = 0;
  float strokeExcursion = 0;
  float strokeTargetPsi = 0;
  float strokeExcSign = 1.0f;
  float strokeStartX = 0;
  float strokeStartPsi = 0;
  float strokePlanned = 0;
  // end

  // filtered copies of the estimate, used for the tolerance decisions.
  // `filtPrimed` says whether the filters have seen a sample yet. ---
  float filtX = 0.0f;
  float filtPsi = 0.0f;
  bool  filtPrimed = false;
  // end

  // measured shift over planned shift, averaged over strokes big enough for
  // the measurement to mean anything. ---
  float strokeGain = 1.0f;
  int strokeCount = 0;
  // end

  // dwell counters. a tolerance has to hold for `dwell` ticks before the
  // supervisor acts on it, so the car does not chatter across the band. ---
  int dwellBad = 0;
  int inTolTicks = 0;
  // end

  // noise measurement. the two accumulators are exponential averages of the
  // squared difference between consecutive estimates of the heading and of the
  // offset. ---
  float noisePsiAcc = 0.0f;
  float noiseXAcc = 0.0f;
  float lastRawPsi = 0.0f;
  float lastRawX = 0.0f;
  float prevRawPsi = 0.0f;
  float prevRawX = 0.0f;
  bool  noisePrimed = false;
  // end

  // calibration. the car is parked and squared up while the samples are taken,
  // one per tick, and their averages become S0 and the trim. ---
  int   calibCount = 0;
  float calibSumS = 0.0f;
  float calibSumT = 0.0f;
  // end

  // the watchdogs. rearFaultTicks counts the unusable rear readings in a row,
  // travel is how far the car has reversed in the whole manoeuvre, and
  // invalidTicks counts the unusable side readings during a stroke. ---
  int   rearFaultTicks = 0;
  float travel = 0.0f;
  int   invalidTicks = 0;
  // end

  // the last usable rear reading, and the travel at the moment it was taken.
  // the rear sensor has a finite range, so an unusable reading does not always
  // mean a broken sensor: the wall can be out of range. how far the car has
  // reversed since that reading says which of the two it is. ---
  float rearSeen = 0.0f;
  float travelAtSeen = 0.0f;
  bool  rearSeenValid = false;
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

  // park the car centred and squared up, then call this once. the samples
  // arrive one per tick and the supervisor keeps taking them until there are
  // enough. S0 folds the bay width and both lateral mount offsets into one
  // number, and the trim takes the left/right mount asymmetry out of the
  // offset. averaging matters here: one reading carries the whole sensor
  // noise into the estimate, and the estimate keeps that bias for the rest
  // of the run.
  void calibrate() {
    calibCount = 0;
    calibSumS = 0.0f;
    calibSumT = 0.0f;
    cfg.S0 = 0.0f;
    cfg.trim = 0.0f;
  }

  // whether enough samples have been taken for the calibration to be good.
  bool calibrated() const { return calibCount >= cfg.calib_samples; }

  // start reversing. the calibration has to be in place first, because S0 is
  // what the heading estimate is measured against.
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

  // one supervisor tick, called at the sensor rate.
  void update() {
    if (hal == nullptr) return;

    const float now = hal->now();
    float dt = (last < 0.0f) ? cfg.dt : (now - last);
    if (dt <= 0.0f) dt = cfg.dt;
    if (dt > 0.5f) dt = 0.5f;
    last = now;

    float gL, gR, gRear;
    hal->readSensors(gL, gR, gRear);

    // the calibration takes a few readings while the car stands parked and
    // squared up, one per tick, and their averages are S0 and the trim. a
    // single reading would put the whole sensor noise into the estimate, and
    // the estimate keeps that bias for the rest of the run. ---
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
    // end

    // there is no pose until the calibration is in place. without S0 the sensor
    // sum has no scale and the heading it reads is nonsense. ---
    geom::Pose p;
    if (calibrated()) p = geom::pose(cfg, gL, gR);
    // end

    // the raw estimate carries a couple of degrees of heading noise, so the
    // tolerance decisions run on a filtered copy. the control law uses the raw
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

    // the tolerances have to be wider than the noise the sensors actually
    // produce, and that is not known until the hardware runs. so it gets
    // measured. the second difference of the estimate is used rather than the
    // first, because the first difference of a car that is turning is mostly
    // yaw and would read as noise. the second difference of a smooth motion
    // is small, so what is left in it is the sensor noise. ---
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
    // end

    // the second difference of white noise has six times the variance of the
    // noise itself, so the variance of the raw estimate is the measured spread
    // divided by six. the filter on the estimate then takes the variance down
    // by a/(2-a) for a first-order filter with coefficient a, and the
    // tolerance decision runs on the filtered estimate, so that is the number
    // the tolerance has to cover. ---
    const float aFilt = 1.0f - expf(-cfg.dt / cfg.tau_filt);
    const float reduce = sqrtf(aFilt / (2.0f - aFilt));
    const float sigmaPsi = sqrtf(fabsf(noisePsiAcc) / 6.0f) * reduce;
    const float sigmaX = sqrtf(fabsf(noiseXAcc) / 6.0f) * reduce;
    float tolAlign = cfg.tol_align;
    float tolOff = cfg.tol_off;
    if (sigmaPsi * 3.0f > tolAlign) tolAlign = sigmaPsi * 3.0f;
    if (sigmaX * 3.0f > tolOff) tolOff = sigmaX * 3.0f;
    // end

    // runtime limits, recomputed every tick from the live geometry. the bay is
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
    // reverse, with the inner loop on the servo. ---
    hal->setMotor(-1);
    travel += driveSpeed(cfg) * dt;

    if (p.ok) {
      const float delta = ctl.step(p.x, p.psi, -driveSpeed(cfg), dt, psiCap);
      servoDeg = 90.0f + delta * 57.29578f;
      hal->setServo(servoDeg);
    }
    // end

    // rear limit. the trigger sits in front of the wanted final gap by the
    // measured stopping distance plus the sensor latency margin. ---
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
        // the rear limit ends the manoeuvre wherever the car happens to be, so
        // say whether that was inside the tolerance. the raw estimate is the
        // one to judge it by: the filtered copy lags the car by the distance it
        // covers in a filter time constant. ---
        tel.outOfSpec = (fabsf(p.x) > cfg.tol_off) ||
                        (fabsf(p.psi) > cfg.tol_align);
        // end
        return;
      }
    } else if (rearSeenValid && rearSeen - (travel - travelAtSeen) > trigger) {
      // the wall is out of the sensor's range, which is not a fault. the car
      // keeps reversing until the wall comes back into range. ---
      rearFaultTicks = 0;
    } else if (++rearFaultTicks >= cfg.rear_fault_ticks) {
      // the wall should be in reach by now and the sensor cannot see it, so the
      // car stops rather than reverse blind. ---
      phase = Phase::GIVEUP;
      stroke = Stroke::NONE;
      hal->setMotor(0);
      tel.rearFault = true;
      return;
    }
    // end

    // the travel watchdog, the backstop for a rear reading that is usable but
    // wrong. there is no odometer, so the distance comes from the commanded
    // speed. ---
    if (travel > cfg.back_in_max_travel) {
      phase = Phase::GIVEUP;
      stroke = Stroke::NONE;
      hal->setMotor(0);
      tel.travelFault = true;
      return;
    }
    // end

    // out of tolerance, so a correction is needed. that only happens once the
    // condition has held for a few ticks, so the car does not chatter in and
    // out of the band while it drifts. ---
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
    // end
  }

  // ======================================================= CORRECT ============
  // run one correction. sub-state 0 evaluates and picks the next correction,
  // heading before offset. sub-state 1 runs the stroke. sub-state 2 settles the
  // car before the next evaluation.
  void stepCorrect(float now, float dt, const geom::Pose& p,
                   const geom::Pose& fp, float gRear, float psiCap,
                   float tolOff, float tolAlign) {
    switch (sub) {
      case 0: {
        if (!p.ok) {
          // no usable measurement. brake and wait rather than guess. ---
          hal->setMotor(0);
          return;
          // end
        }
        const bool headBad = fabsf(fp.psi) > tolAlign;
        const bool offBad = fabsf(fp.x) > tolOff;

        if (!headBad && !offBad) {
          // in tolerance. hold it for a few ticks before going back to
          // reversing, so the car does not chatter between the phases. ---
          if (++inTolTicks >= cfg.dwell) {
            phase = Phase::BACK_IN;
            sub = 0;
            stroke = Stroke::NONE;
            inTolTicks = 0;
            dwellBad = 0;
            ctl.reset();
          }
          return;
          // end
        }

        inTolTicks = 0;

        // still out of tolerance. if the budget is spent, stop and flag it. ---
        if (strokeCount >= cfg.max_strokes) {
          phase = Phase::GIVEUP;
          stroke = Stroke::NONE;
          hal->setMotor(0);
          return;
        }
        // end

        // heading first. a standing heading error makes the offset grow, and a
        // translate stroke is only worth planning from a heading that is
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
          // the switch is triggered by the measured heading excursion, not by a
          // timer, so the excursion is the same whatever the speed does. ---
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
        const int strokeDir = (cfg.stroke_dir > 0) ? 1 : -1;
        hal->setMotor(strokeDir);

        bool done = (t >= strokeDur);
        bool abort = false;

        // a stroke that drives towards the wall is watched the way the reverse
        // is. the strokes normally run forward, so this only bites if the
        // config points them the other way. ---
        bool rearHit = false;
        if (strokeDir < 0) {
          travel += driveSpeed(cfg) * dt;
          const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
          if (geom::validRange(gRear) && gRear <= trigger) rearHit = true;
        }
        // end

        // a stroke that has lost the side readings has to stop: the car may be
        // against a wall, and holding the lock makes that worse. ---
        if (p.ok) {
          invalidTicks = 0;
        } else if (++invalidTicks >= cfg.stroke_invalid_ticks) {
          abort = true;
        }
        // end

        if (p.ok) {
          const float excursion = p.psi - strokeStartPsi;

          if (stroke == Stroke::ROTATE) {
            // closed loop on the heading change: stop when the car has rotated
            // by the amount that was asked for. ---
            const float change = p.psi - strokeStartPsi;
            if (strokeTargetPsi > 0.0f ? change >= strokeTargetPsi - 0.010f
                                       : change <= strokeTargetPsi + 0.010f) {
              done = true;
            }
            // end
          } else {
            if (strokeLeg == 0) {
              // first leg: run until the planned excursion is reached. ---
              if (strokeExcSign * excursion >= strokeExcursion) {
                strokeLeg = 1;
                strokeLegT0 = now;
              }
              // end
            } else {
              // second leg: steer back until the heading is where it started,
              // and a little before that, because the servo needs time to come
              // back to centre and the car keeps yawing while it does. the
              // compensation is 0.7 of what the slew model says, because the
              // model overestimates the yaw left by about 2.4 times. ---
              if (strokeExcSign * excursion <=
                  0.7f * slewOvershoot(fabsf(strokeDelta)))
                done = true;
              // end
            }
          }

        }

        // clearance abort: never let the body swing past what the bay holds.
        // the filtered heading is used, because it is the last heading that was
        // measured, and the interlock has to work when the measurement does
        // not. ---
        if (fabsf(fp.psi) > psiCap + 0.087f) abort = true;
        // end

        if (abort || rearHit || t > cfg.stroke_timeout) done = true;

        if (done) {
          hal->setMotor(0);
          hal->setServo(90.0f);
          servoDeg = 90.0f;

          // learn from the result: the shift a stroke actually produced against
          // the shift the plan asked for. this absorbs whatever the model gets
          // wrong, plus the surface, the speed and the servo. ---
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
          if (rearHit) {
            // the rear limit while correcting ends the manoeuvre, the same way
            // it does while reversing. ---
            phase = Phase::DONE;
            sub = 0;
            tel.outOfSpec = (fabsf(p.x) > cfg.tol_off) ||
                            (fabsf(p.psi) > cfg.tol_align);
            // end
          } else {
            sub = 2;
          }
        }
        break;
      }

      case 2: {
        // settle: brakes on, wheels centred, wait for the car to be still. ---
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
  // both stroke relations are exact and independent of speed and ramp. they
  // come from dx/dpsi = -(L/tan delta) * sin(psi), which holds at every instant
  // whatever the speed happens to be, so a stroke lands where it is aimed even
  // if the battery sags.
  //
  //   translate S-curve:  |dx|   = 2 * L * (1 - cos(psi_peak)) / tan(delta)
  //   rotate drag:        |drag| =     L * (1 - cos(dpsi))    / tan(delta)
  //
  // the peak excursion of a translate stroke sets the displacement, and the
  // stroke is stopped on the measured excursion rather than on a timer, which
  // is what makes it speed independent.
  //
  // two things stand between the plan and the floor. the servo deadband means
  // the wheel angle is the command less the deadband, and the effective angle
  // is what the relations need. the servo slew means the car keeps yawing for
  // a moment after the command flips to the opposite lock, so the peak
  // overshoots the trigger. both are modelled, and whatever is left over is
  // absorbed by a gain the supervisor learns from the measured result of each
  // stroke.

  // the largest equivalent angle the steering can deliver. with Ackermann
  // steering that is what the inner wheel can reach, which is less than the
  // wheel's own stop. ---
  float steerLimit() {
    if (cfg.steer_model == SteerModel::ACKERMANN) return ack::deltaEqMax(cfg);
    return cfg.delta_max;
  }
  // end

  // the stroke amplitude for an offset, limited by what the steering can
  // deliver. ---
  float strokeAmp(float offset) {
    float a = (fabsf(offset) <= cfg.fine_threshold) ? cfg.delta_fine
                                                    : cfg.delta_coarse;
    const float lim = steerLimit();
    if (a > lim) a = lim;
    return a;
  }
  // end

  // the steer angle the wheels actually reach for a given command.
  float effAmp(float amp) {
    float a = amp - cfg.servo_deadband;
    if (a < 0.05f) a = 0.05f;
    return a;
  }

  // rough time for a heading excursion, used only as a safety limit.
  float excursionTime(float dpsi, float amp) {
    const float tanD = tanf(effAmp(amp));
    const float v = driveSpeed(cfg);
    if (tanD < 1e-6f || v < 1e-3f) return cfg.stroke_timeout;
    return fabsf(dpsi) * car::L_wb / (v * tanD);
  }

  // how far past the trigger the peak excursion lands, because the servo needs
  // time to swing to the opposite lock.
  float slewOvershoot(float amp) {
    const float tanD = tanf(effAmp(amp));
    const float yawRate = cfg.speed * tanD / car::L_wb;
    return yawRate * (effAmp(amp) / cfg.servo_rate);
  }

  void beginRotate(const geom::Pose& p, float psiCap, float now) {
    // aim to null the heading. the rotation is split into steps the measurement
    // can resolve, and the peak yaw is capped by the clearance. ---
    float limit = psiCap;
    if (limit > cfg.psi_meas_max) limit = cfg.psi_meas_max;
    // end

    // the heading change being asked for: towards zero, and no more than the
    // cap allows in one step. ---
    float want = -p.psi;
    if (want > limit) want = limit;
    if (want < -limit) want = -limit;
    // end

    const float amp = strokeAmp(p.psi);

    // the heading change follows the sign of the steering angle and the
    // direction of travel: sign(dpsi) = -sign(cmd) * dir. ---
    const float dir = (cfg.stroke_dir > 0) ? 1.0f : -1.0f;
    const float delta = -((want >= 0.0f) ? 1.0f : -1.0f) * dir * amp;
    // end

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

    // the first leg always swings the heading in the direction that moves the
    // car towards the centreline, so the sign of the first leg's command is
    // fixed, and so is the direction the excursion goes in. the heading change
    // follows sign(dpsi) = -sign(cmd) * dir. ---
    const float first = (p.x > 0.0f) ? amp : -amp;
    const float sgnFirst = (first > 0.0f) ? 1.0f : -1.0f;
    const float excSign = -sgnFirst * dir;
    // end

    // the tangent carries the sign of the first leg, because the shift a stroke
    // makes is 2(L/tan delta)(cos psi_peak - cos psi_start), and the bracket is
    // negative for any excursion that grows. the sign of the tangent is
    // therefore what decides which way the car goes, and a lock to the left
    // shifts it to the left. an unsigned tangent plans every stroke the same
    // way, and the strokes planned for the far side of the centreline move the
    // car further away from it instead of back towards it. ---
    const float tanD = sgnFirst * tanf(effAmp(amp));
    // end

    // the shift of a symmetric S-curve that returns the heading to where it
    // started is
    //   dx = 2 * (L / tan(delta)) * (cos(psi_peak) - cos(psi_start))
    // so the plan works in the peak heading, not in the excursion. that
    // matters: an excursion measured from a start that is already yawed can
    // land the peak near zero, where the shift is almost insensitive to it. ---
    const float cs = cosf(p.psi);
    // end

    // the peak is capped, which caps the shift. the cap also keeps the
    // absolute heading inside the cone-safe window, so the measurement stays
    // good enough for the whole stroke: past the edge of the cone the reading
    // bends by 1/cos, and the worst that bend does to the heading estimate over
    // the whole grid is a tenth of a degree, which is a seventh of the noise
    // the tolerances already absorb. ---
    float cap = psiCap;
    if (cap > cfg.psi_meas_max) cap = cfg.psi_meas_max;
    // end

    // how much room the swing has inside that window. swinging away from the
    // centreline runs out of window, swinging through it has the far side of
    // the window to use. ---
    const bool away = (excSign * p.psi >= 0.0f);
    float room = away ? (cap - fabsf(p.psi)) : (cap + fabsf(p.psi));
    if (room < 0.0f) room = 0.0f;
    // end

    // the largest shift available: put the peak at the cap. its sign follows
    // the sign of the tangent, so it points the same way as the shift the
    // stroke is able to make. ---
    float dxMax = 2.0f * (car::L_wb / tanD) * (cosf(cap) - cs) * strokeGain;
    if (away && room <= 0.0f) dxMax = 0.0f;
    // end

    // the shift the stroke is asked for, towards the centreline, limited by
    // what one stroke can do. ---
    const float dxWant = -p.x;
    float dx = dxWant;
    if (fabsf(dx) > fabsf(dxMax)) dx = dxMax;
    // end

    // the peak that produces that shift. the gain is what previous strokes
    // actually achieved against what the model predicted, so the model is asked
    // for a proportionally larger excursion to land on the target. ---
    float c = cs + (dx / strokeGain) * tanD / (2.0f * car::L_wb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    const float peakMag = acosf(c);
    // end

    // two candidate peaks, and the excursion has to go in the forced direction.
    // the first candidate peaks at +peakMag, the second at -peakMag. ---
    const float excA = peakMag - p.psi;
    const float excB = -peakMag - p.psi;
    float exc;
    if (excSign * excA > 0.0f) {
      exc = excA;
    } else if (excSign * excB > 0.0f) {
      exc = excB;
    } else {
      // neither works, so take the smaller swing.
      exc = (fabsf(excA) <= fabsf(excB)) ? excA : excB;
    }
    // end

    // the peak lands past the trigger, because the servo needs time to swing to
    // the opposite lock. ask for a smaller excursion to land on target. ---
    float target = fabsf(exc) - slewOvershoot(amp);
    if (target < 0.02f) target = 0.02f;
    // end

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
    // ramp the servo back to centre rather than snapping it. ---
    const float step = 180.0f * dt;
    if (servoDeg > 90.0f + step) servoDeg -= step;
    else if (servoDeg < 90.0f - step) servoDeg += step;
    else servoDeg = 90.0f;
    hal->setServo(servoDeg);
    // end
  }
};
