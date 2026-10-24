// Compile check for the control core. Geometry, loop and supervisor against a plain config, with no HAL attached. A signature change breaks the build here instead of changing behaviour on the car.

#include "supervisor.h"

// The results have to land somewhere the optimiser cannot remove, or the calls that produce them are dropped before they compile.
volatile float controlBuildSink = 0.0f;

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
