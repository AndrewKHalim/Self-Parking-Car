// direct checks of the sensor geometry.
//
// the simulator cannot cover the clearance interlock: the cap only binds in a
// bay narrower than the car, and in such a bay the side sensors lose the wall
// before the cap matters. so the geometry is checked here instead, where the
// numbers can be stated rather than swept for.
//
//   g++ -std=c++17 -O2 -Wall -Wextra -Icontrol -o build/geom_checks
//       tools/geom_checks.cpp -lm
//
// every check prints a line and sets the exit status, so a failure is visible
// in a build log.

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

  // the sensor reports 8190 when it sees nothing and 65535 on a timeout.
  // neither is a distance. ---
  check("validRange rejects the no-target reading",
        !geom::validRange(8.190f), "8190 mm");
  check("validRange rejects the timeout reading",
        !geom::validRange(65.535f), "65535 mm");
  check("validRange rejects a zero reading", !geom::validRange(0.0f), "0 mm");
  check("validRange accepts a normal gap", geom::validRange(0.25f), "250 mm");
  // end

  // an unusable side reading leaves the clearance unknown, and an unknown
  // clearance must not open the yaw up. ---
  const float unknown = geom::clearance(cfg, 0.25f, 8.190f);
  check("clearance is unknown when a side reading is bad",
        unknown < 0.0f, "negative means unknown");

  const float known = geom::clearance(cfg, 0.25f, 0.18f);
  check("clearance is the tighter side", known > 0.121f && known < 0.124f,
        "the 58 mm body offset comes off");
  // end

  // the cap has to fall to the small end when the clearance is unknown, and it
  // has to bind below the cone-safe limit when the clearance is small. ---
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
  // end

  // the estimate has to recover the pose it was built from. the gaps are built
  // from the geometry in geometry.h, so this is a round trip through the sensor
  // model and back. ---
  for (float psi = -0.15f; psi <= 0.151f; psi += 0.05f) {
    for (float x = -0.05f; x <= 0.051f; x += 0.025f) {
      const float ux = sinf(psi);
      const float lx = -cosf(psi);
      const float gL = 0.5f * 0.5f + x + car::d_L * ux + cfg.e_L * lx;
      const float gR = 0.5f * 0.5f - (x - car::d_R * ux - cfg.e_R * lx);

      Config c = cfg;
      // the calibration is taken with the car centred and squared up, so S0 is
      // the sensor sum in that pose and the trim is the half difference of the
      // two readings in it. ---
      c.S0 = 0.5f - (cfg.e_L + cfg.e_R);
      c.trim = 0.5f * (cfg.e_R - cfg.e_L);
      // end
      const geom::Pose p = geom::pose(c, gL, gR);

      // the heading comes out of the sum as the sine of the angle, so the
      // estimate is a first-order one: the error at the top of the cone-safe
      // window is a hundredth of a degree. ---
      const bool ok = p.ok && fabsf(p.x - x) < 1e-4f
                      && fabsf(p.psi - psi) < 1e-3f;
      char detail[128];
      snprintf(detail, sizeof(detail), "x=%.1fmm psi=%.1fdeg recovered %.1fmm "
               "%.1fdeg", x * 1000.0f, psi * 57.29578f, p.x * 1000.0f,
               p.psi * 57.29578f);
      check("the estimate recovers the pose", ok, detail);
    }
  }
  // end

  printf("\n%s\n", failures == 0 ? "all checks passed"
                                : "there were failures");
  return failures == 0 ? 0 : 1;
}
