// Compile check for the control core. Geometry and the supervisor against a
// plain config, with no HAL attached. A signature change breaks the build here
// instead of changing behaviour on the car.

#include "supervisor.h"

// The results have to land somewhere the optimiser cannot remove, or the calls
// that produce them are dropped before they compile.
volatile float controlBuildSink = 0.0f;

void controlBuildCheck() {
  Config cfg;
  Supervisor sup;
  sup.cfg = cfg;

  const geom::Pose p = geom::pose(cfg, 0.25f, 0.25f);
  const float clear = geom::clearance(cfg, 0.25f, 0.25f);

  sup.start();
  sup.update();

  controlBuildSink = p.x + p.psi + clear + sup.tel.x;
}
