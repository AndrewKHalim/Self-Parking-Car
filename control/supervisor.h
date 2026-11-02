#pragma once

#include <cmath>

#include "config.h"
#include "geometry.h"
#include "hal.h"

// The supervisor decides what the car is doing, in fixed time segments. It
// carries no steering law. The car reverses straight, then runs one forward
// pulse at a fixed lock and one reverse pulse at the opposite lock, and repeats
// that up to corrections_max times. Fixed times were chosen over a computed
// profile because a computed profile needs a measured car to be right, and this
// car is not measured yet.

enum class Phase { IDLE, CALIBRATE, BACK_IN, PULL, DONE, GIVEUP };

struct Telemetry {
  Phase phase = Phase::IDLE;
  // The stroke model is gone, so this is always zero and the CSV column keeps
  // its place for a future maneuver.
  uint8_t stroke = 0;
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
  bool calFault = false;
  bool calibrated = false;
  bool outOfSpec = false;
};

struct Supervisor {
  // A copy would dangle, because the HAL is injected by attach().
  Supervisor() = default;
  Supervisor(const Supervisor&) = delete;
  Supervisor& operator=(const Supervisor&) = delete;

  Config cfg;
  Hal* hal = nullptr;

  Phase phase = Phase::IDLE;

  // PULL runs three fixed segments. sub counts them, and each segment records
  // the instant it started so its length is a subtraction and not a tick count.
  int sub = 0;
  float phaseT0 = 0;
  float subT0 = 0;
  float pullSign = 1.0f;

  // Calibration. One valid pair per tick while the car stands still, and the
  // averages become S0 and the trim when the batch is full.
  int calibCount = 0;
  float calibSumS = 0;
  float calibSumT = 0;

  // The reverse leg. travel is dead reckoning from the commanded speed, and it
  // is the backstop for a rear reading that is usable but wrong.
  float travel = 0;
  int corrections = 0;
  int rearFaultTicks = 0;
  float rearSeen = 0;
  float travelAtSeen = 0;
  bool rearSeenValid = false;

  // The pull direction comes from the side pair at the start of the pull. When
  // the pair drops out the last usable sign is held.
  float lastSign = 1.0f;

  float last = -1.0f;
  float servoDeg = 90.0f;
  float peakYaw = 0.0f;
  Telemetry tel;

  void attach(Hal* h) { hal = h; }

  // Whether the batch of samples S0 is averaged from is full.
  bool calibrated() const { return calibCount >= cfg.calib_samples; }

  // Begin a run. The calibration is retaken here rather than reused from boot,
  // because whatever pose the car was powered on in is not necessarily the pose
  // it is started from.
  void start() {
    phase = Phase::CALIBRATE;
    phaseT0 = (hal != nullptr) ? hal->now() : 0.0f;
    sub = 0;
    subT0 = phaseT0;
    corrections = 0;
    travel = 0.0f;
    rearFaultTicks = 0;
    rearSeen = 0.0f;
    travelAtSeen = 0.0f;
    rearSeenValid = false;
    lastSign = 1.0f;
    peakYaw = 0.0f;
    servoDeg = servo::CENTRE_DEG;
    last = -1.0f;
    tel.rearFault = false;
    tel.travelFault = false;
    tel.calFault = false;
    tel.outOfSpec = false;
    beginCalibration();
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

    // The pose is reported and nothing else. No motor or servo decision in this
    // file reads it, because the maneuver no longer closes a loop on either.
    geom::Pose p;
    if (calibrated()) p = geom::pose(cfg, gL, gR);

    tel.t = now;
    tel.x = p.x;
    tel.psi = p.psi;
    tel.rear = gRear;
    tel.gL = gL;
    tel.gR = gR;
    tel.valid = p.ok;
    tel.clearance = geom::clearance(cfg, gL, gR);
    if (p.ok && fabsf(p.psi) > peakYaw) peakYaw = fabsf(p.psi);
    tel.peakYaw = peakYaw;

    switch (phase) {
      case Phase::IDLE:
        hal->setMotor(0);
        hal->setServo(servo::CENTRE_DEG);
        servoDeg = servo::CENTRE_DEG;
        // The page shows psi before the first run, so the car calibrates on
        // boot as well. Every start retakes it.
        accumulateCalibration(gL, gR);
        break;

      case Phase::CALIBRATE:
        stepCalibrate(now, gL, gR);
        break;

      case Phase::BACK_IN:
        stepBackIn(dt, now, gL, gR, gRear);
        break;

      case Phase::PULL:
        stepPull(now, gRear);
        break;

      case Phase::DONE:
      case Phase::GIVEUP:
        hal->setMotor(0);
        centreSteering(dt);
        // A run that could not calibrate leaves the page's start button disabled
        // until the flag comes back, so the parked car keeps sampling. A normal
        // run has a full batch already and this does nothing.
        accumulateCalibration(gL, gR);
        // Reported only. A run that ends outside the band is flagged for the
        // log, and the numbers are the ones the page already shows.
        if (phase == Phase::DONE && p.ok) {
          tel.outOfSpec = fabsf(p.x) > cfg.tol_off || fabsf(p.psi) > cfg.tol_align;
        }
        break;
    }

    tel.phase = phase;
    tel.servo = servoDeg;
    tel.travel = travel;
    tel.strokes = corrections;
    tel.calibrated = calibrated();
  }

  // ======================================================= CALIBRATE =========
  void stepCalibrate(float now, float gL, float gR) {
    hal->setMotor(0);
    hal->setServo(servo::CENTRE_DEG);
    servoDeg = servo::CENTRE_DEG;

    accumulateCalibration(gL, gR);
    if (calibrated()) {
      enterBackIn(now);
      return;
    }

    // A car that never sees both walls has no heading to work from, so the run
    // gives up instead of reversing on a guess.
    if (elapsed(now, phaseT0, cfg.calib_timeout_ms)) {
      phase = Phase::GIVEUP;
      hal->setMotor(0);
      tel.calFault = true;
    }
  }

  // ======================================================= BACK_IN ===========
  void stepBackIn(float dt, float now, float gL, float gR, float gRear) {
    // Straight reverse. The servo is held at centre every tick and no reading
    // steers the car, which is what keeps the wheels still until the car moves.
    const bool nearWall = geom::validRange(gRear) && gRear < cfg.crawl_below_m;
    hal->setMotor(-1, nearWall ? cfg.crawl_duty : cfg.reverse_duty);
    hal->setServo(servo::CENTRE_DEG);
    servoDeg = servo::CENTRE_DEG;

    travel += cfg.speed * dt;

    // main steered its correction opposite the larger gap, and that comparison
    // is the whole direction decision here.
    if (geom::validRange(gL) && geom::validRange(gR)) {
      lastSign = ((gL - gR) > 0.0f) ? 1.0f : -1.0f;
    }

    // Rear limit. The trigger sits ahead of the wanted final gap by the assumed
    // stopping distance and the latency margin.
    const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
    if (geom::validRange(gRear)) {
      rearSeen = gRear;
      travelAtSeen = travel;
      rearSeenValid = true;
      rearFaultTicks = 0;
      if (gRear <= trigger) {
        hal->setMotor(0);
        if (corrections < cfg.corrections_max) {
          beginPull(now);
        } else {
          phase = Phase::DONE;
        }
        return;
      }
    } else if (rearSeenValid && rearSeen - (travel - travelAtSeen) > trigger) {
      // Out of range is not a fault, so keep reversing until the wall returns.
      rearFaultTicks = 0;
    } else if (++rearFaultTicks >= cfg.rear_fault_ticks) {
      // The wall should be in reach and the sensor cannot see it.
      phase = Phase::GIVEUP;
      hal->setMotor(0);
      tel.rearFault = true;
      return;
    }

    // Travel watchdog, the backstop for a rear reading that is usable but wrong,
    // and the clock that stops a run whose sensor has gone.
    if (travel > cfg.back_in_max_travel ||
        elapsed(now, phaseT0, cfg.back_in_max_ms)) {
      phase = Phase::GIVEUP;
      hal->setMotor(0);
      tel.travelFault = true;
    }
  }

  // ======================================================= PULL ==============
  void stepPull(float now, float gRear) {
    switch (sub) {
      case 1: {
        // Forward at a fixed lock until the clock runs out, or until the car is
        // clear of the wall the way main left it.
        servoDeg = clampServo(servo::CENTRE_DEG - pullSign * cfg.pull_angle_deg);
        hal->setServo(servoDeg);
        hal->setMotor(1, cfg.stroke_duty);
        const bool clear = geom::validRange(gRear) && gRear > cfg.pull_rear_exit_m;
        if (clear || elapsed(now, subT0, cfg.pull_forward_ms)) {
          sub = 2;
          subT0 = now;
        }
        break;
      }

      case 2: {
        // The return leg. The same lock the other way, reversing, for the same
        // fixed time, which is the S-curve the maneuver asks for.
        servoDeg = clampServo(servo::CENTRE_DEG + pullSign * cfg.pull_angle_deg);
        hal->setServo(servoDeg);
        hal->setMotor(-1, cfg.reverse_duty);
        if (elapsed(now, subT0, cfg.pull_reverse_ms)) {
          sub = 3;
          subT0 = now;
        }
        break;
      }

      case 3: {
        // Brake with the wheels straight, and let the car settle before the next
        // reverse leg measures anything.
        hal->setMotor(0);
        hal->setServo(servo::CENTRE_DEG);
        servoDeg = servo::CENTRE_DEG;
        if (elapsed(now, subT0, cfg.settle_ms)) {
          corrections++;
          enterBackIn(now);
        }
        break;
      }
    }
  }

  // ======================================================= helpers ===========
  static bool elapsed(float now, float t0, uint32_t ms) {
    return now - t0 >= 0.001f * (float)ms;
  }

  static float clampServo(float deg) {
    if (deg < servo::MIN_DEG) return servo::MIN_DEG;
    if (deg > servo::MAX_DEG) return servo::MAX_DEG;
    return deg;
  }

  void centreSteering(float dt) {
    // Ramp the servo back to centre rather than snapping it.
    const float step = 180.0f * dt;
    if (servoDeg > servo::CENTRE_DEG + step) servoDeg -= step;
    else if (servoDeg < servo::CENTRE_DEG - step) servoDeg += step;
    else servoDeg = servo::CENTRE_DEG;
    hal->setServo(servoDeg);
  }

  void beginCalibration() {
    calibCount = 0;
    calibSumS = 0.0f;
    calibSumT = 0.0f;
    cfg.S0 = 0.0f;
    cfg.trim = 0.0f;
  }

  // One pair per tick, while the batch is short. A full batch becomes S0 and the
  // trim, and the average is what keeps one noisy pair out of the whole run.
  void accumulateCalibration(float gL, float gR) {
    if (calibrated()) return;
    if (!geom::validRange(gL) || !geom::validRange(gR)) return;
    calibSumS += gL + gR;
    calibSumT += gL - gR;
    calibCount++;
    if (calibCount >= cfg.calib_samples) {
      const float n = (float)calibCount;
      cfg.S0 = calibSumS / n;
      cfg.trim = 0.5f * calibSumT / n;
    }
  }

  void enterBackIn(float now) {
    phase = Phase::BACK_IN;
    phaseT0 = now;
    travel = 0.0f;
    rearFaultTicks = 0;
    rearSeen = 0.0f;
    travelAtSeen = 0.0f;
    rearSeenValid = false;
  }

  void beginPull(float now) {
    phase = Phase::PULL;
    sub = 1;
    subT0 = now;
    phaseT0 = now;
    pullSign = lastSign * cfg.pull_steer_sign;
  }
};
