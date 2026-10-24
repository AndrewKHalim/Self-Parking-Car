// Step-response rig for the lateral position loop. It steps the rear axle offset x while the car reverses at driveSpeed(cfg). The controller is the real Control::step with its xRef hook, and the plant is the bicycle and servo model from tools/sim_main.cpp. The drive ramp is left out, since accel is a simulator field.
// g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/step_rig tools/step_rig.cpp -lm

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>

#include "ackermann.h"
#include "config.h"
#include "control.h"

namespace {

constexpr float kDeg = 3.14159265f / 180.0f;

struct State {
  float x = 0.0f;
  float psi = 0.0f;
  float v = 0.0f;
  float delta = 0.0f;
  float travel = 0.0f;
};

// One servo tick, using the model from tools/sim_main.cpp.
void servoStep(float cmd, float& wheel, float dt, float deadband, float rate) {
  const float err = cmd - wheel;
  if (fabsf(err) > deadband) {
    float mv = err - (err > 0.0f ? deadband : -deadband);
    const float lim = rate * dt;
    if (fabsf(mv) > lim) mv = (mv > 0.0f ? lim : -lim);
    wheel += mv;
  }
}

// One plant tick, covering servo, rear axle kinematics and parallel scrub.
void plantStep(const Config& c, State& s, float cmd, float dt) {
  servoStep(cmd, s.delta, dt, c.servo_deadband, c.servo_rate);

  float yaw;
  if (c.steer_model == SteerModel::ACKERMANN) {
    const float din = ack::deltaIn(c, s.delta);
    const float dout = ack::deltaOut(c, s.delta);
    const float ci = 1.0f / tanf(din);
    const float co = 1.0f / tanf(dout);
    yaw = -s.v * 2.0f / (car::L_wb * (ci + co));
  } else {
    yaw = -s.v * tanf(s.delta) / car::L_wb;
  }
  s.psi += yaw * dt;
  s.x += s.v * sinf(s.psi) * dt;
  s.travel += fabsf(s.v) * dt;

  if (c.steer_model == SteerModel::PARALLEL) {
    const float slip = fabsf(s.v) * fabsf(tanf(s.delta)) * car::t_front
                     / (2.0f * car::L_wb);
    const float sgn = (s.v > 0.0f) ? 1.0f : ((s.v < 0.0f) ? -1.0f : 0.0f);
    s.v -= sgn * c.scrub_gain * slip * dt;
  }
}

}

int main(int argc, char** argv) {
  Config cfg;
  float refM = 0.040f;
  float tStep = 1.0f;
  float tEnd = 15.0f;
  float noiseM = 0.0f;
  float speedMps = 0.0f;
  unsigned seed = 1;
  std::string out = "docs/step_response.csv";
  std::string servoOut = "build/step_servo.csv";
  std::string infoOut = "build/step_run_info.txt";

  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    auto next = [&]() -> float {
      return (i + 1 < argc) ? atof(argv[++i]) : 0.0f;
    };
    if (a == "--ref") refM = next();
    else if (a == "--t-step") tStep = next();
    else if (a == "--duration") tEnd = next();
    else if (a == "--noise") noiseM = next();
    else if (a == "--speed") speedMps = next();
    else if (a == "--seed") seed = (unsigned)next();
    else if (a == "--out") out = argv[++i];
    else if (a == "--servo-out") servoOut = argv[++i];
    else if (a == "--info-out") infoOut = argv[++i];
    else if (a == "--help") {
      printf("usage: step_rig [--ref m] [--t-step s] [--duration s]\n"
             "                [--noise m] [--seed n] [--speed mps]\n"
             "                [--out csv] [--servo-out csv]\n"
             "                [--info-out txt]\n");
      return 0;
    }
  }

  for (const std::string* p : {&out, &servoOut, &infoOut}) {
    std::error_code ec;
    const auto parent = std::filesystem::path(*p).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);
  }

  // The parking loop runs at the configured sensor period with the plant sub-stepped at 1 ms, and the controller sees the nominal speed.
  if (speedMps > 0.0f) cfg.speed = speedMps;
  const float dt = cfg.dt;
  const float vNom = -driveSpeed(cfg);
  const float psiCap = cfg.psi_cap_max;
  const int subN = (int)lroundf(dt / 0.001f);
  const float subDt = dt / (float)subN;

  State s;
  s.v = vNom;

  Control ctl;
  ctl.c = &cfg;
  ctl.reset();

  std::mt19937 rng(seed);
  std::normal_distribution<float> gauss(0.0f, 1.0f);

  FILE* f = fopen(out.c_str(), "w");
  if (f == nullptr) {
    fprintf(stderr, "cannot write %s\n", out.c_str());
    return 1;
  }
  fprintf(f, "t_s,ref_m,x_meas_m,x_true_m,psi_rad,steer_cmd_rad,"
             "wheel_rad,v_mps,travel_m,integ_m_s,error_m\n");

  const int ticks = (int)lroundf(tEnd / dt);
  for (int n = 0; n <= ticks; n++) {
    const float t = (float)n * dt;
    const float ref = (t >= tStep) ? refM : 0.0f;
    const float xMeas = s.x + noiseM * gauss(rng);
    const float cmd = ctl.step(xMeas, s.psi, vNom, dt, psiCap, ref);

    fprintf(f, "%.4f,%.6f,%.7f,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.7f,%.7f\n",
            t, ref, xMeas, s.x, s.psi, cmd, s.delta, s.v, s.travel, ctl.integ,
            xMeas - ref);

    for (int k = 0; k < subN; k++) plantStep(cfg, s, cmd, subDt);
  }
  fclose(f);

  // Servo-only step traces. The amplitude is a test input rather than a servo parameter, and the small one is half the deadband, so it shows the dead zone.
  const float bigCmd = 20.0f * kDeg;
  const float smallCmd = 0.5f * cfg.servo_deadband;
  float bigWheel = 0.0f;
  float smallWheel = 0.0f;
  FILE* g = fopen(servoOut.c_str(), "w");
  if (g == nullptr) {
    fprintf(stderr, "cannot write %s\n", servoOut.c_str());
    return 1;
  }
  fprintf(g, "t_s,big_cmd_deg,big_wheel_deg,small_cmd_deg,small_wheel_deg\n");
  const float servoDt = 0.001f;
  const int servoN = (int)lroundf(2.0f / servoDt);
  for (int n = 0; n <= servoN; n++) {
    const float t = (float)n * servoDt;
    const float big = (t >= 0.2f) ? bigCmd : 0.0f;
    const float small = (t >= 0.2f) ? smallCmd : 0.0f;
    servoStep(big, bigWheel, servoDt, cfg.servo_deadband, cfg.servo_rate);
    servoStep(small, smallWheel, servoDt, cfg.servo_deadband,
              cfg.servo_rate);
    fprintf(g, "%.4f,%.4f,%.4f,%.4f,%.4f\n", t, big / kDeg, bigWheel / kDeg,
            small / kDeg, smallWheel / kDeg);
  }
  fclose(g);

  FILE* h = fopen(infoOut.c_str(), "w");
  if (h == nullptr) {
    fprintf(stderr, "cannot write %s\n", infoOut.c_str());
    return 1;
  }
  fprintf(h, "ref_m=%.6f\nt_step_s=%.4f\nduration_s=%.4f\ndt_s=%.4f\n"
             "noise_m=%.6f\nseed=%u\nspeed_mps=%.6f\npsi_cap_rad=%.6f\n"
             "servo_deadband_deg=%.4f\nservo_rate_deg_s=%.4f\n"
             "delta_max_deg=%.4f\nomega_n=%.4f\nzeta=%.4f\nki=%.4f\n"
             "steer=%s\n",
          refM, tStep, tEnd, dt, noiseM, seed, fabsf(vNom), psiCap,
          cfg.servo_deadband / kDeg, cfg.servo_rate / kDeg,
          cfg.delta_max / kDeg, cfg.omega_n, cfg.zeta, cfg.ki,
          (cfg.steer_model == SteerModel::ACKERMANN) ? "ackermann"
                                                     : "parallel");
  fclose(h);

  printf("wrote %s\nwrote %s\nwrote %s\n", out.c_str(), servoOut.c_str(),
         infoOut.c_str());
  return 0;
}
