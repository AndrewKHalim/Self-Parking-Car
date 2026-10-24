// compile check for the control core.
//
// the firmware includes the core through src/esp32_hal.h, so this is no longer
// the only way it reaches the target compiler. it stays as a check that the
// entry points compile and link with no hal attached and nothing running: the
// geometry, the loop and the supervisor against a plain config. a signature
// that changes under main.cpp's feet shows up here as a build error rather than
// as a behaviour change on the car.

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
