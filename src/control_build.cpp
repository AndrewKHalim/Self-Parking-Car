// Compile check for the control core. The supervisor against a plain config,
// with no HAL attached. A signature change breaks the build here instead of
// changing behaviour on the car.

#include "supervisor.h"

// The results have to land somewhere the optimiser cannot remove, or the calls
// that produce them are dropped before they compile.
volatile float controlBuildSink = 0.0f;

void controlBuildCheck() {
  Supervisor sup;
  sup.start();
  sup.update();
  controlBuildSink = sup.tel.rear + sup.tel.servo;
}
