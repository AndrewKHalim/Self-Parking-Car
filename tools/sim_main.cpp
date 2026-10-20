// Host-side simulator for the reverse-parking controller.
//
// The plant is a kinematic bicycle model referred to the rear axle. The sensor
// forward model casts to the bay wall planes and then applies the same
// conversion the firmware applies, including the 25 degree cone limit of the
// VL53L0X, so the sweep shows where the heading estimate degrades.
//
// The controller code is the same code the firmware runs. The control/ folder
// has no hardware calls, only the Hal interface, and that is the whole point
// of the split (PLAN.md section 14).
//
//   g++ -std=c++17 -O2 -Icontrol -o build/sim tools/sim_main.cpp -lm

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include "config.h"
#include "control.h"
#include "geometry.h"
#include "hal.h"
#include "supervisor.h"

static const float kDeg = 3.14159265f / 180.0f;

// Half the VL53L0X field of view. ---
static const float kConeHalf = 12.5f * kDeg;
// end

struct Sim : Hal {
  Config cfg;

  // The bay, in metres. ---
  float W = 0.50f;
  float D = 0.60f;
  float rearBumper = 0.05f;
  // end

  // Car state. x is the rear axle offset from the bay centreline, psi the
  // heading, and y the rear axle distance from the rear wall. v is the speed
  // along the car's own forward axis and is signed, and delta is the steer
  // angle the wheels have actually reached. ---
  float x = 0.0f;
  float psi = 0.0f;
  float y = 0.0f;
  float v = 0.0f;
  float delta = 0.0f;
  float cmdDelta = 0.0f;
  int motorDir = 0;
  // end

  // Sensor cache, refreshed at the ranging period. ---
  float gL = 0.0f, gR = 0.0f, gRear = 0.0f;
  // end

  float t = 0.0f;

  // Statistics collected over the run. ---
  float minClear = 1e9f;
  float minRear = 1e9f;
  float peakPsi = 0.0f;
  bool hitWall = false;
  // end

  // Model parameters. The deadband and the slew rate are the servo's, and the
  // two accelerations describe the drive. ---
  float noise = 0.003f;
  float deadband = 1.5f * kDeg;
  float servoRate = 300.0f * kDeg;
  float accel = 0.5f;
  float brakeDecel = 1.0f;
  // end

  std::mt19937 rng{1};
  std::normal_distribution<float> gauss{0.0f, 1.0f};

  // =========================================================== Hal ============
  float now() override { return t; }

  void readSensors(float& a, float& b, float& c) override {
    a = gL; b = gR; c = gRear;
  }

  void setServo(float deg) override { cmdDelta = (deg - 90.0f) * kDeg; }

  void setMotor(int dir) override { motorDir = dir; }

  // ========================================================= plant ============
  void step(float dt) {
    // The servo is rate limited and has a deadband. The wheels only move once
    // the command clears the deadband, and then they lag it by that much. ---
    float err = cmdDelta - delta;
    if (fabsf(err) > deadband) {
      float mv = err - (err > 0 ? deadband : -deadband);
      float lim = servoRate * dt;
      if (fabsf(mv) > lim) mv = (mv > 0 ? lim : -lim);
      delta += mv;
    }
    // end

    // The drive is on or off with a ramp, and the brake decelerates to a stop
    // and stays there. ---
    float target = (motorDir > 0) ? cfg.speed : (motorDir < 0 ? -cfg.speed : 0.0f);
    if (motorDir == 0) {
      float d = brakeDecel * dt;
      if (fabsf(v) <= d) v = 0.0f;
      else v -= (v > 0 ? d : -d);
    } else {
      float a = accel * dt;
      if (v < target) { v += a; if (v > target) v = target; }
      else { v -= a; if (v < target) v = target; }
    }
    // end

    // Kinematics at the rear axle. The car's forward axis points out of the
    // bay, so reversing drives it in towards the rear wall. ---
    psi += -v * tanf(delta) / cfg.L_wb * dt;
    x += v * sinf(psi) * dt;
    y += v * cosf(psi) * dt;

    float apsi = fabsf(psi);
    if (apsi > peakPsi) peakPsi = apsi;
    // end
  }

  // True perpendicular gaps, with no cone factor and no noise. ---
  void trueGaps(float& left, float& right, float& rear) const {
    const float uX = sinf(psi);
    const float lX = -cosf(psi);
    const float lxp = x + cfg.d_L * uX + cfg.e_L * lX;
    const float rxp = x - cfg.d_R * uX - cfg.e_R * lX;
    left = lxp + 0.5f * W;
    right = 0.5f * W - rxp;
    rear = y - rearBumper * cosf(psi);
  }
  // end

  // Sample the sensors, including the cone limit and the range failures.
  void updateSensors() {
    float lt, rt, rr;
    trueGaps(lt, rt, rr);

    // Past the half-cone the sensor starts to see along the wall, so the
    // reading grows. ---
    float excess = fabsf(psi) - kConeHalf;
    if (excess < 0.0f) excess = 0.0f;
    const float cone = 1.0f / cosf(excess);
    lt *= cone;
    rt *= cone;
    // end

    // The body clearance is what the walls actually see. ---
    const float clear = fminf(lt - cfg.e_out_L, rt - cfg.e_out_R);
    if (clear < minClear) minClear = clear;
    if (rr < minRear) minRear = rr;
    if (clear <= 0.0f || rr <= 0.0f) hitWall = true;
    // end

    // Range limits: below the cover glass minimum or past the target range,
    // the sensor reports no target at all. ---
    auto inRange = [](float d) { return d > 0.03f && d < 2.0f; };
    gL = inRange(lt) ? lt + gauss(rng) * noise : 8.190f;
    gR = inRange(rt) ? rt + gauss(rng) * noise : 8.190f;
    gRear = inRange(rr) ? rr + gauss(rng) * noise : 8.190f;
    // end
  }
};

struct Result {
  bool timedOut = false;
  bool gaveUp = false;
  bool hitWall = false;
  float finalX = 0, finalPsi = 0, finalRear = 0;
  float minClear = 0, minRear = 0, peakPsi = 0;
  int strokes = 0;
  int strokesUsed = 0;
  bool centred = false, aligned = false, rearOk = false;
};

static Result runScenario(const Config& base, float x0, float psi0, float W,
                          float D, float v, float noise, unsigned seed,
                          float tmax = 40.0f, bool trace = false) {
  Config cfg = base;
  cfg.speed = v;

  Sim sim;
  sim.cfg = cfg;
  sim.W = W;
  sim.D = D;
  sim.noise = noise;
  sim.rng.seed(seed);

  Supervisor sup;
  sup.cfg = cfg;
  sup.attach(&sim);
  sup.trace = trace;

  // Calibration step: the car is parked centred and squared up, and the
  // controller captures S0 and the zero-offset trim. ---
  sim.x = 0.0f;
  sim.psi = 0.0f;
  sim.y = 0.5f * D;
  sim.updateSensors();
  sup.calibrate();
  // end

  // Then the car is placed at the test pose, offset and angled. ---
  sim.x = x0;
  sim.psi = psi0;
  sim.y = D - 0.08f;
  sim.v = 0.0f;
  sim.delta = 0.0f;
  sup.start();
  sup.last = -1.0f;
  // end

  // The plant runs far faster than the sensors, so the plant takes a small
  // step and the controller runs at the ranging period. ---
  const float plantDt = 0.001f;
  const float sensorPeriod = 0.033f;
  float sensorT = 0.0f;
  int lastStrokes = 0;
  // end

  Result r;
  while (sim.t < tmax) {
    sim.step(plantDt);
    sim.t += plantDt;

    sensorT += plantDt;
    if (sensorT >= sensorPeriod) {
      sensorT = 0.0f;
      sim.updateSensors();
      sup.update();

      // Print the truth next to the estimate whenever a stroke is running,
      // so the estimate's error is visible. ---
      if (trace && sup.stroke != Stroke::NONE) {
        float lt, rt, rr;
        sim.trueGaps(lt, rt, rr);
        printf("      t=%.2f leg=%d true psi=%6.2f deg  true x=%6.1f mm  "
               "est psi=%6.2f  est x=%6.1f\n",
               sim.t, sup.strokeLeg, sim.psi * 57.29578f, sim.x * 1000.0f,
               sup.tel.psi * 57.29578f, sup.tel.x * 1000.0f);
      }
      // end

      // Print again each time a stroke completes. ---
      if (sup.strokeCount != lastStrokes) {
        lastStrokes = sup.strokeCount;
        if (trace) {
          float lt, rt, rr;
          sim.trueGaps(lt, rt, rr);
          printf("  [t=%.2fs] true x=%.1fmm psi=%.2fdeg  est x=%.1fmm "
                 "psi=%.2fdeg\n",
                 sim.t, sim.x * 1000.0f, sim.psi * 57.29578f,
                 sup.tel.x * 1000.0f, sup.tel.psi * 57.29578f);
        }
      }
      // end
    }

    if (sup.phase == Phase::DONE || sup.phase == Phase::GIVEUP) break;
  }

  float lt, rt, rr;
  sim.trueGaps(lt, rt, rr);
  r.finalX = sup.tel.x;
  r.finalPsi = sup.tel.psi;
  r.finalRear = rr;
  r.minClear = sim.minClear;
  r.minRear = sim.minRear;
  r.peakPsi = sim.peakPsi;
  r.strokes = sup.strokeCount;
  r.gaveUp = (sup.phase == Phase::GIVEUP);
  r.timedOut = (sup.phase != Phase::DONE && sup.phase != Phase::GIVEUP);
  r.hitWall = sim.hitWall;

  // The demo is judged on where the car actually ended up, not on what the
  // noisy estimate said at the last sample. ---
  r.centred = fabsf(sim.x) <= cfg.tol_off;
  r.aligned = fabsf(sim.psi) <= cfg.tol_align;
  r.rearOk = (r.finalRear >= 0.015f && r.finalRear <= 0.100f);
  // end
  return r;
}

int main(int argc, char** argv) {
  Config cfg;
  float x0 = 0.04f, psi0 = 5.0f * kDeg, W = 0.50f, D = 0.60f;
  float v = cfg.speed, noise = 0.003f;
  unsigned seed = 1;
  bool csv = false;
  bool trace = false;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> float { return (i + 1 < argc) ? atof(argv[++i]) : 0.0f; };
    if (a == "--x") x0 = next();
    else if (a == "--psi") psi0 = next() * kDeg;
    else if (a == "--W") W = next();
    else if (a == "--D") D = next();
    else if (a == "--v") v = next();
    else if (a == "--noise") noise = next();
    else if (a == "--seed") seed = (unsigned)next();
    else if (a == "--csv") csv = true;
    else if (a == "--trace") trace = true;
    else if (a == "--tol-off") cfg.tol_off = next();
    else if (a == "--tol-align") cfg.tol_align = next() * kDeg;
    else if (a == "--omega") cfg.omega_n = next();
    else if (a == "--zeta") cfg.zeta = next();
    else if (a == "--d-stop") cfg.d_stop = next();
  }

  Result r = runScenario(cfg, x0, psi0, W, D, v, noise, seed, 40.0f, trace);

  if (csv) {
    printf("x0,psi0,W,D,v,noise,seed,finalX,finalPsi,finalRear,minClear,"
           "minRear,peakPsi,strokes,centred,aligned,rearOk,hitWall,gaveUp,"
           "timedOut\n");
    printf("%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%u,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,%d,"
           "%d,%d,%d,%d,%d,%d\n",
           x0, psi0 / kDeg, W, D, v, noise, seed, r.finalX, r.finalPsi / kDeg,
           r.finalRear, r.minClear, r.minRear, r.peakPsi / kDeg, r.strokes,
           r.centred, r.aligned, r.rearOk, r.hitWall, r.gaveUp, r.timedOut);
  } else {
    printf("scenario: x0=%.1fcm psi0=%.1fdeg bay=%.0fx%.0fcm v=%.2f noise=%.1fmm\n",
           x0 * 100, psi0 / kDeg, W * 100, D * 100, v, noise * 1000);
    printf("  final   x=%.1fmm psi=%.2fdeg rear=%.0fmm\n", r.finalX * 1000,
           r.finalPsi / kDeg, r.finalRear * 1000);
    printf("  strokes %d   peak yaw %.1fdeg   min clearance %.0fmm\n",
           r.strokes, r.peakPsi / kDeg, r.minClear * 1000);
    printf("  centred=%d aligned=%d rear_ok=%d hit_wall=%d gave_up=%d "
           "timeout=%d\n",
           r.centred, r.aligned, r.rearOk, r.hitWall, r.gaveUp, r.timedOut);
  }
  return 0;
}
