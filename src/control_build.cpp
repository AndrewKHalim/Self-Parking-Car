// compile check for the control core.
//
// control/ is header only and the firmware never includes it, so on the target
// nothing there reaches the compiler. this translation unit exists to change
// that: it includes supervisor.h and calls the inline entry points once, which
// is enough for the compiler to emit them. nothing here runs, and there is no
// Hal implementation yet, so this proves the core compiles for the ESP32-S3,
// not that the loop works.

#include "supervisor.h"

// the results have to land somewhere the optimiser cannot remove, or the calls
// that produce them are dropped before they compile. ---
volatile float controlBuildSink = 0.0f;
// end

void controlBuildCheck() {
  Config cfg;
  Control ctl;
  ctl.c = &cfg;

  Supervisor sup;
  sup.cfg = cfg;

  const geom::Pose p = geom::pose(cfg, 0.25f, 0.25f);
  const float clear = geom::clearance(cfg, 0.25f, 0.25f);
  const float cap = geom::peakYawCap(cfg, clear);
  const float delta = ctl.step(p.x, p.psi, -driveSpeed(cfg), cfg.dt, cap);
  const float din = ack::deltaIn(cfg, delta);
  const float dout = ack::deltaOut(cfg, delta);
  const float deq = ack::deltaEq(cfg, din, dout);
  const float limit = ack::deltaEqMax(cfg);

  controlBuildSink = delta + deq + limit + sup.tel.x;
  (void)sup.calibrated();
  (void)sup.start();
  sup.update();
}
