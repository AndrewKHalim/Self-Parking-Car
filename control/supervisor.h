#pragma once

#include <cmath>

#include "config.h"
#include "hal.h"

// The supervisor is one fixed-time maneuver and nothing else. The car reverses
// straight, then runs one forward pulse at a fixed lock and one reverse pulse at
// the opposite lock, and repeats that up to corrections_max times. No pose, no
// filter and no feedback steering: this car did not move under a feedback law,
// and this one moves on duty alone. Fixed times were chosen over a computed
// profile because a computed profile needs a measured car to be right.

enum class Phase { IDLE, BACK_IN, PULL, DONE, GIVEUP };

// Why a run stopped before DONE. NONE is the running and the parked state.
enum class Fault { NONE, REAR, TIMEOUT };

inline const char* phaseName(Phase p) {
  switch (p) {
    case Phase::IDLE: return "idle";
    case Phase::BACK_IN: return "back_in";
    case Phase::PULL: return "pull";
    case Phase::DONE: return "done";
    case Phase::GIVEUP: return "giveup";
  }
  return "unknown";
}

inline const char* faultName(Fault f) {
  switch (f) {
    case Fault::NONE: return "none";
    case Fault::REAR: return "rear";
    case Fault::TIMEOUT: return "timeout";
  }
  return "unknown";
}

struct Telemetry {
  Phase phase = Phase::IDLE;
  Fault fault = Fault::NONE;
  float t = 0;
  // The pose is gone. x and psi stay on the wire at zero so the page and the
  // CSV keep their shape, and no decision reads them.
  float x = 0;
  float psi = 0;
  float rear = 0;
  float gL = 0;
  float gR = 0;
  float servo = 90;
  int strokes = 0;
  // True when the rear reading on this tick was usable.
  bool valid = false;
};

struct Supervisor {
  // A copy would dangle, because the HAL is injected by attach().
  Supervisor() = default;
  Supervisor(const Supervisor&) = delete;
  Supervisor& operator=(const Supervisor&) = delete;

  Config cfg;
  Hal* hal = nullptr;

  Phase phase = Phase::IDLE;

  // PULL runs two fixed segments. sub counts them, and each segment records the
  // instant it started so its length is a subtraction and not a tick count.
  // end
  int sub = 0;
  float phaseT0 = 0;
  float subT0 = 0;
  float pullSign = 1.0f;

  // The pull direction comes from the sign of gL - gR on the last valid side
  // pair, held when the pair drops out. main steered its correction opposite
  // the larger gap, and that comparison is the whole direction decision.
  // end
  float pairSign = 1.0f;

  // The reverse leg. A rear reading that is invalid does not end the leg, so
  // there is no fault clock here; the leg clock is the only stop.
  int corrections = 0;

  float servoDeg = 90.0f;
  Telemetry tel;

  void attach(Hal* h) { hal = h; }

  // Begin a run. There is no gate, so a start always begins the reverse on the
  // next tick.
  void start() {
    corrections = 0;
    pairSign = 1.0f;
    tel.fault = Fault::NONE;
    enterBackIn((hal != nullptr) ? hal->now() : 0.0f);
  }

  // One supervisor tick, called at the sensor rate.
  void update() {
    if (hal == nullptr) return;

    const float now = hal->now();

    float gL, gR, gRear;
    hal->readSensors(gL, gR, gRear);

    tel.t = now;
    tel.rear = gRear;
    tel.gL = gL;
    tel.gR = gR;
    tel.valid = rearValidReading(gRear);

    switch (phase) {
      case Phase::IDLE:
      case Phase::DONE:
      case Phase::GIVEUP:
        hal->setMotor(0);
        hal->setServo(servo::CENTRE_DEG);
        servoDeg = servo::CENTRE_DEG;
        break;
      case Phase::BACK_IN:
        stepBackIn(now, gL, gR, gRear);
        break;
      case Phase::PULL:
        stepPull(now);
        break;
    }

    tel.phase = phase;
    tel.servo = servoDeg;
    tel.strokes = corrections;
  }

 private:
  static bool elapsed(float now, float t0, uint32_t ms) {
    return now - t0 >= 0.001f * (float)ms;
  }

  void enterBackIn(float now) {
    phase = Phase::BACK_IN;
    phaseT0 = now;
    sub = 0;
  }

  void beginPull(float now) {
    phase = Phase::PULL;
    sub = 1;
    subT0 = now;
    pullSign = pairSign * cfg.pull_steer_sign;
  }

  void stepBackIn(float now, float gL, float gR, float gRear) {
    const bool rearOk = rearValidReading(gRear);

    // The lock is latched from the last usable side pair. When a side drops out
    // the last sign is held, so a single bad sample cannot steer the pull.
    // end
    if (validReading(gL) && validReading(gR)) {
      pairSign = (gL - gR >= 0.0f) ? 1.0f : -1.0f;
    }

    hal->setServo(servo::CENTRE_DEG);
    servoDeg = servo::CENTRE_DEG;

    // The trigger adds the roll-on and the latency margin to the wanted gap, so
    // the motor is cut before the tail reaches the target. A reading of exactly
    // 0 is the sensor's minimum-range status at the wall, which main treated as
    // close, so it falls below the trigger and stops the leg too.
    // end
    const float trigger = cfg.rear_target + cfg.d_stop + cfg.rear_margin;
    if (rearOk && gRear <= trigger) {
      hal->setMotor(0);
      if (corrections < cfg.corrections_max) {
        beginPull(now);
      } else {
        phase = Phase::DONE;
      }
      return;
    }

    // An invalid rear reading never ends the leg by itself. The car keeps
    // reversing because the observed failure was a dead rear sensor at boot,
    // and giving up there leaves the car parked. The leg clock is the only stop
    // and the fault names why: rear when the wall was never visible, timeout
    // when it was but the car still did not reach it.
    // end
    if (elapsed(now, phaseT0, cfg.back_in_max_ms)) {
      hal->setMotor(0);
      tel.fault = rearOk ? Fault::TIMEOUT : Fault::REAR;
      phase = Phase::GIVEUP;
      return;
    }

    hal->setMotor(-1, cfg.reverse_duty);
  }

  void stepPull(float now) {
    switch (sub) {
      case 1:
        // Forward at a fixed lock, the way main pulled forward to correct.
        servoDeg = servo::CENTRE_DEG - pullSign * cfg.pull_angle_deg;
        hal->setServo(servoDeg);
        hal->setMotor(1, cfg.stroke_duty);
        if (elapsed(now, subT0, cfg.pull_forward_ms)) {
          sub = 2;
          subT0 = now;
        }
        break;

      case 2:
        // The return leg. The same lock the other way, reversing for the same
        // fixed time. This leg is also the settle, so the next reverse starts
        // from a car that has already stopped moving forward.
        // end
        servoDeg = servo::CENTRE_DEG + pullSign * cfg.pull_angle_deg;
        hal->setServo(servoDeg);
        hal->setMotor(-1, cfg.reverse_duty);
        if (elapsed(now, subT0, cfg.pull_reverse_ms)) {
          corrections++;
          enterBackIn(now);
        }
        break;
    }
  }
};
