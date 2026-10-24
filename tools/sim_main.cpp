// Host-side simulator for the reverse-parking controller. A kinematic bicycle model at the rear axle, and a sensor forward model that casts to the bay walls with the same conversion as the firmware, cone limit included. The controller is the firmware's own code, and control/ has no hardware calls.
// g++ -std=c++17 -O2 -Icontrol -o build/sim tools/sim_main.cpp -lm

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include "ackermann.h"
#include "config.h"
#include "control.h"
#include "geometry.h"
#include "hal.h"
#include "supervisor.h"

static const float kDeg = 3.14159265f / 180.0f;

// Half the VL53L0X field of view.
static const float kConeHalf = 12.5f * kDeg;

struct Sim : Hal {
  Config cfg;

  // The bay, in metres.
  float W = 0.50f;
  float D = 0.60f;
  float rearBumper = 0.05f;

  // Car state. x is rear axle offset, psi heading, y distance from the rear wall, v signed speed along the forward axis, delta the wheel angle reached.
  float x = 0.0f;
  float psi = 0.0f;
  float y = 0.0f;
  float v = 0.0f;
  float delta = 0.0f;
  float cmdDelta = 0.0f;
  int motorDir = 0;

  // Sensor cache, refreshed at the ranging period.
  float gL = 0.0f, gR = 0.0f, gRear = 0.0f;

  float t = 0.0f;

  // Statistics collected over the run.
  float minClear = 1e9f;
  float minRear = 1e9f;
  float peakPsi = 0.0f;
  float travel = 0.0f;
  bool hitWall = false;

  // Fault injection, so the watchdogs can be tested. The time is when the fault starts, and a negative time means it never starts.
  float faultRearAt = -1.0f;
  float faultSideAt = -1.0f;

  // Servo deadband and slew come from the config, so plant and controller describe the same servo. The two accelerations describe the drive.
  float noise = 0.003f;
  float accel = 0.5f;
  float brakeDecel = 1.0f;


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
    // The servo is rate limited and has a deadband. The wheels only move once the command clears the deadband, and then they lag it by that much.
    float err = cmdDelta - delta;
    if (fabsf(err) > cfg.servo_deadband) {
      float mv = err - (err > 0 ? cfg.servo_deadband : -cfg.servo_deadband);
      float lim = cfg.servo_rate * dt;
      if (fabsf(mv) > lim) mv = (mv > 0 ? lim : -lim);
      delta += mv;
    }

    // The drive is on or off with a ramp. The brake decelerates to a stop and stays there.
    const float vMax = driveSpeed(cfg);
    float target = (motorDir > 0) ? vMax : (motorDir < 0 ? -vMax : 0.0f);
    if (motorDir == 0) {
      float d = brakeDecel * dt;
      if (fabsf(v) <= d) v = 0.0f;
      else v -= (v > 0 ? d : -d);
    } else {
      float a = accel * dt;
      if (v < target) { v += a; if (v > target) v = target; }
      else { v -= a; if (v < target) v = target; }
    }

    // Kinematics at the rear axle. The forward axis points out of the bay, so reversing drives the car in towards the rear wall. Both steering models give the same radius, since a fixed rear axle turns about that line. They differ in the scrub, not the geometry.
    float yaw;
    if (cfg.steer_model == SteerModel::ACKERMANN) {
      const float din = ack::deltaIn(cfg, delta);
      const float dout = ack::deltaOut(cfg, delta);
      const float ci = 1.0f / tanf(din);
      const float co = 1.0f / tanf(dout);
      yaw = -v * 2.0f / (car::L_wb * (ci + co));
    } else {
      yaw = -v * tanf(delta) / car::L_wb;
    }
    psi += yaw * dt;
    x += v * sinf(psi) * dt;
    y += v * cosf(psi) * dt;

    // Parallel steering drags the front wheels sideways in every turn.
    if (cfg.steer_model == SteerModel::PARALLEL) {
      const float slip = fabsf(v) * fabsf(tanf(delta)) * car::t_front
                       / (2.0f * car::L_wb);
      const float sgn = (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f);
      v -= sgn * cfg.scrub_gain * slip * dt;
    }

    float apsi = fabsf(psi);
    if (apsi > peakPsi) peakPsi = apsi;

    travel += fabsf(v) * dt;
  }

  // True perpendicular gaps, with no cone factor and no noise.
  void trueGaps(float& left, float& right, float& rear) const {
    const float uX = sinf(psi);
    const float lX = -cosf(psi);
    const float lxp = x + car::d_L * uX + cfg.e_L * lX;
    const float rxp = x - car::d_R * uX - cfg.e_R * lX;
    left = lxp + 0.5f * W;
    right = 0.5f * W - rxp;
    rear = y - rearBumper * cosf(psi);
  }

  // Sample the sensors, including the cone limit and the range failures.
  void updateSensors() {
    float lt, rt, rr;
    trueGaps(lt, rt, rr);

    // Past the half-cone the sensor starts to see along the wall, so the reading grows.
    float excess = fabsf(psi) - kConeHalf;
    if (excess < 0.0f) excess = 0.0f;
    const float cone = 1.0f / cosf(excess);
    lt *= cone;
    rt *= cone;

    // The body clearance is what the walls actually see.
    const float clear = fminf(lt - car::e_out_L, rt - car::e_out_R);
    if (clear < minClear) minClear = clear;
    if (rr < minRear) minRear = rr;
    if (clear <= 0.0f || rr <= 0.0f) hitWall = true;

    // Range limits. Below the cover glass minimum or past the target range, the sensor reports no target at all.
    auto inRange = [](float d) { return d > 0.03f && d < 2.0f; };
    gL = inRange(lt) ? lt + gauss(rng) * noise : 8.190f;
    gR = inRange(rt) ? rt + gauss(rng) * noise : 8.190f;
    gRear = inRange(rr) ? rr + gauss(rng) * noise : 8.190f;

    if (faultSideAt >= 0.0f && t >= faultSideAt) {
      gL = 8.190f;
      gR = 8.190f;
    }
    if (faultRearAt >= 0.0f && t >= faultRearAt) {
      gRear = 8.190f;
    }
  }
};

struct Result {
  bool timedOut = false;
  bool gaveUp = false;
  bool hitWall = false;
  bool rearFault = false;
  bool travelFault = false;
  bool outOfSpec = false;
  float finalX = 0, finalPsi = 0, finalRear = 0;
  float minClear = 0, minRear = 0, peakPsi = 0, travel = 0, reverse = 0;
  int strokes = 0;
  int strokesUsed = 0;
  bool centred = false, aligned = false, rearOk = false;
};

static Result runScenario(const Config& base, float x0, float psi0, float W,
                          float D, float v, float noise, unsigned seed,
                          float tmax = 40.0f, bool trace = false,
                          float faultRearAt = -1.0f, float faultSideAt = -1.0f,
                          float calX = 0.0f, float calPsi = 0.0f) {
  Config cfg = base;
  cfg.speed = v;

  Sim sim;
  sim.cfg = cfg;
  sim.W = W;
  sim.D = D;
  sim.noise = noise;
  sim.brakeDecel = driveModel(cfg.motor_driver).brakeDecel;
  sim.rng.seed(seed);
  sim.faultRearAt = faultRearAt;
  sim.faultSideAt = faultSideAt;

  Supervisor sup;
  sup.cfg = cfg;
  sup.attach(&sim);
  sup.trace = trace;

  // Calibration. The car is parked centred and squared up while the controller averages a batch of readings into S0 and the trim, one per tick.
  sim.x = calX;
  sim.psi = calPsi;
  sim.y = 0.5f * D;
  sup.calibrate();
  int calibTicks = 0;
  while (!sup.calibrated() && calibTicks < 500) {
    sim.updateSensors();
    sup.update();
    calibTicks++;
  }
  if (!sup.calibrated()) {
    // The side readings never came up, so there is nothing to calibrate against and the run cannot start.
    Result r;
    r.timedOut = true;
    return r;
  }

  // Then the car is placed at the test pose, offset and angled.
  sim.x = x0;
  sim.psi = psi0;
  sim.y = D - 0.08f;
  sim.v = 0.0f;
  sim.delta = 0.0f;
  sup.start();
  sup.last = -1.0f;

  // The plant runs far faster than the sensors, so it takes small steps while the controller runs at the ranging period.
  const float plantDt = 0.001f;
  const float sensorPeriod = 0.033f;
  float sensorT = 0.0f;
  int lastStrokes = 0;

  Result r;
  while (sim.t < tmax) {
    sim.step(plantDt);
    sim.t += plantDt;

    sensorT += plantDt;
    if (sensorT >= sensorPeriod) {
      sensorT = 0.0f;
      sim.updateSensors();
      sup.update();

      // Print the truth next to the estimate while a stroke runs, so the error is visible.
      if (trace && sup.stroke != Stroke::NONE) {
        float lt, rt, rr;
        sim.trueGaps(lt, rt, rr);
        printf("      t=%.2f leg=%d true psi=%6.2f deg  true x=%6.1f mm  "
               "est psi=%6.2f  est x=%6.1f\n",
               sim.t, sup.strokeLeg, sim.psi * 57.29578f, sim.x * 1000.0f,
               sup.tel.psi * 57.29578f, sup.tel.x * 1000.0f);
      }

      // Print again each time a stroke completes.
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
  r.rearFault = sup.tel.rearFault;
  r.travelFault = sup.tel.travelFault;
  r.outOfSpec = sup.tel.outOfSpec;
  r.travel = sim.travel;
  r.reverse = sup.tel.travel;

  // The demo is judged on where the car actually ended up, not on what the noisy estimate said at the last sample.
  r.centred = fabsf(sim.x) <= cfg.tol_off;
  r.aligned = fabsf(sim.psi) <= cfg.tol_align;
  r.rearOk = (r.finalRear >= 0.015f && r.finalRear <= 0.100f);
  return r;
}

int main(int argc, char** argv) {
  Config cfg;
  float x0 = 0.04f, psi0 = 5.0f * kDeg, W = 0.50f, D = 0.60f;
  float v = cfg.speed, noise = 0.003f;
  unsigned seed = 1;
  bool csv = false;
  bool trace = false;
  float faultRearAt = -1.0f, faultSideAt = -1.0f;
  float calX = 0.0f, calPsi = 0.0f;
  std::string steer = "parallel", driver = "drv8833";

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
    else if (a == "--steer") steer = argv[++i];
    else if (a == "--driver") driver = argv[++i];
    else if (a == "--fault-rear-at") faultRearAt = next();
    else if (a == "--cal-x") calX = next();
    else if (a == "--cal-psi") calPsi = next() * kDeg;
    else if (a == "--fault-side-at") faultSideAt = next();
  }

  if (steer == "ackermann") cfg.steer_model = SteerModel::ACKERMANN;
  if (driver == "lm298") cfg.motor_driver = MotorDriver::LM298;

  Result r = runScenario(cfg, x0, psi0, W, D, v, noise, seed, 40.0f, trace,
                         faultRearAt, faultSideAt, calX, calPsi);

  if (csv) {
    printf("x0,psi0,W,D,v,noise,seed,finalX,finalPsi,finalRear,minClear,"
           "minRear,peakPsi,travel,reverse,strokes,centred,aligned,rearOk,"
           "hitWall,gaveUp,timedOut,rearFault,travelFault,outOfSpec\n");
    printf("%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%u,%.4f,%.3f,%.4f,%.4f,%.4f,%.3f,"
           "%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           x0, psi0 / kDeg, W, D, v, noise, seed, r.finalX, r.finalPsi / kDeg,
           r.finalRear, r.minClear, r.minRear, r.peakPsi / kDeg, r.travel,
           r.reverse, r.strokes, r.centred, r.aligned, r.rearOk, r.hitWall,
           r.gaveUp, r.timedOut, r.rearFault, r.travelFault, r.outOfSpec);
  } else {
    printf("scenario: x0=%.1fcm psi0=%.1fdeg bay=%.0fx%.0fcm v=%.2f noise=%.1fmm "
           "steer=%s driver=%s\n",
           x0 * 100, psi0 / kDeg, W * 100, D * 100, v, noise * 1000,
           (cfg.steer_model == SteerModel::ACKERMANN) ? "ackermann" : "parallel",
           (cfg.motor_driver == MotorDriver::LM298) ? "lm298" : "drv8833");
    printf("  final   x=%.1fmm psi=%.2fdeg rear=%.0fmm\n", r.finalX * 1000,
           r.finalPsi / kDeg, r.finalRear * 1000);
    printf("  strokes %d   peak yaw %.1fdeg   min clearance %.0fmm   "
           "travel %.2fm   reverse %.2fm\n",
           r.strokes, r.peakPsi / kDeg, r.minClear * 1000, r.travel, r.reverse);
    printf("  centred=%d aligned=%d rear_ok=%d hit_wall=%d gave_up=%d "
           "timeout=%d rear_fault=%d travel_fault=%d out_of_spec=%d\n",
           r.centred, r.aligned, r.rearOk, r.hitWall, r.gaveUp, r.timedOut,
           r.rearFault, r.travelFault, r.outOfSpec);
  }
  return 0;
}
