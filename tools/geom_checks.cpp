// Direct checks of the sensor geometry. The simulator cannot cover the clearance interlock, so the numbers are stated here rather than swept for.
// g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/geom_checks tools/geom_checks.cpp -lm

#include <cmath>
#include <cstdio>

#include "config.h"
#include "geometry.h"

static int failures = 0;

static void check(const char* what, bool ok, const char* detail) {
  printf("%-46s %s   %s\n", what, ok ? "ok  " : "FAIL", detail);
  if (!ok) failures++;
}

int main() {
  Config cfg;

  // The sensor reports 8190 with no target and 65535 on a timeout.
  check("validRange rejects the no-target reading",
        !geom::validRange(8.190f), "8190 mm");
  check("validRange rejects the timeout reading",
        !geom::validRange(65.535f), "65535 mm");
  check("validRange rejects a zero reading", !geom::validRange(0.0f), "0 mm");
  check("validRange accepts a normal gap", geom::validRange(0.25f), "250 mm");

  // An unusable side reading leaves the clearance unknown, and an unknown clearance must not open the yaw up.
  const float unknown = geom::clearance(cfg, 0.25f, 8.190f);
  check("clearance is unknown when a side reading is bad",
        unknown < 0.0f, "negative means unknown");

  const float known = geom::clearance(cfg, 0.25f, 0.18f);
  check("clearance is the tighter side", known > 0.121f && known < 0.124f,
        "the 58 mm body offset comes off");

  // The cap falls to the small end on an unknown clearance and binds below the cone-safe limit on a small one.
  const float capUnknown = geom::peakYawCap(cfg, unknown);
  check("the yaw cap fails closed", capUnknown == cfg.psi_cap_min,
        "the small end of the range");

  const float capTight = geom::peakYawCap(cfg, 0.040f);
  check("the yaw cap binds on a tight clearance",
        capTight < cfg.psi_meas_max && capTight > cfg.psi_cap_min,
        "below the cone-safe limit");

  const float capRoomy = geom::peakYawCap(cfg, 0.180f);
  check("the yaw cap opens up with room", capRoomy == cfg.psi_cap_max,
        "the clearance is not what binds in a normal bay");

  // Round trip. Gaps built from geometry.h, then the pose recovered from them.
  for (float psi = -0.15f; psi <= 0.151f; psi += 0.05f) {
    for (float x = -0.05f; x <= 0.051f; x += 0.025f) {
      const float ux = sinf(psi);
      const float lx = -cosf(psi);
      const float gL = 0.5f * 0.5f + x + car::d_L * ux + cfg.e_L * lx;
      const float gR = 0.5f * 0.5f - (x - car::d_R * ux - cfg.e_R * lx);

      Config c = cfg;
      // Calibration is taken with the car centred and squared up. S0 is the sensor sum in that pose, and the trim is half the difference.
      c.S0 = 0.5f - (cfg.e_L + cfg.e_R);
      c.trim = 0.5f * (cfg.e_R - cfg.e_L);
      const geom::Pose p = geom::pose(c, gL, gR);

      // The heading comes out of the sum as the sine, a first-order estimate worth a hundredth of a degree of error at the cone edge.
      const bool ok = p.ok && fabsf(p.x - x) < 1e-4f
                      && fabsf(p.psi - psi) < 1e-3f;
      char detail[128];
      snprintf(detail, sizeof(detail), "x=%.1fmm psi=%.1fdeg recovered %.1fmm "
               "%.1fdeg", x * 1000.0f, psi * 57.29578f, p.x * 1000.0f,
               p.psi * 57.29578f);
      check("the estimate recovers the pose", ok, detail);
    }
  }

  printf("\n%s\n", failures == 0 ? "all checks passed"
                                : "there were failures");
  return failures == 0 ? 0 : 1;
}
