#pragma once

#include <cstdint>

// Every tunable for the car, and the one file to edit after measuring. A
// measured value carries the date it was taken and the gate that produced it,
// and every value still guessed names the gate that replaces it. PLAN.md
// section 13 lists what is left to measure.
// The order runs the chassis geometry, the maneuver, the board pins and the
// task timing.

// Chassis dimensions in metres, 2026-10-29. L_car and L_wb are given for this
// project, and V0 A1 and A2 replace them. t_front is the kingpin bore spacing in
// Base.step, 90.5 mm, and not a tyre centre, because no wheels are modelled
// there, so V0 A3 replaces it with the tyre centres. The body is 119.5 mm wide
// at the two rear pods only, and 84.5 mm wide through the middle.
// d_L and d_R are the side-sensor stations from the rear axle, measured
// 2026-11-02 against the rear wheel centre. The left face is 67 mm ahead of it
// and the right face 147 mm ahead, so the right is the forward sensor and the
// faces are 80 mm apart. The first tape, 105 mm from the left face to the rear
// sensor, reads 87 mm from these two, so one of the three was taken to a
// different point and the axle measurement is the one kept.
// d_L is the left station ahead of the axle. d_R is the right station negated,
// which is why it is negative here, and that keeps the divisor d_L + d_R equal
// to the signed separation the sum divides by, 80 mm in magnitude. The pose
// keeps its sign either way because the divisor and the sum flip together.
// Both faces sit 95 mm apart across the car, so they are not flush with the
// 119.5 mm widest point, and V0 D4 and D5 take e_out from the built mounts.
// L_wb and t_front are axle and bore centres, not body dimensions.
namespace car {
constexpr float L_wb    = 0.183f;
constexpr float t_front = 0.0905f;
constexpr float L_car   = 0.234f;
constexpr float d_L     = 0.067f;
constexpr float d_R     = -0.147f;
constexpr float e_out_L = 0.000f;
constexpr float e_out_R = 0.000f;
}
// end

// The steering linkage. The servo stands upright with the module 0.8, 12 tooth
// pinion on its spline, and that pinion drives the rack, so one servo degree is
// 0.0838 mm of rack. The steering arm is 14.00 mm from the kingpin to the tie
// rod, so one servo degree is 0.343 degrees at the road wheel. Only the
// converter below still uses the ratio, because the firmware commands servo
// degrees directly now. Both figures are from the CAD, which has no servo mount
// and no tie rod, so the ratio covers the pinion and the arm only. The centre
// and the sign are still the assumed 1500 us and left. V0 B1 and B2 replace all
// three.
// MIN_DEG and MAX_DEG are the range main drove and clamped to. The linkage is
// only assumed, so every write is held inside it rather than at the servo's own
// 0 to 180.
namespace servo {
constexpr float CENTRE_DEG = 90.0f;
constexpr float MIN_DEG    = 45.0f;
constexpr float MAX_DEG    = 135.0f;
constexpr float DIR        = 1.0f;
constexpr float PER_WHEEL  = 2.9f;
}
// end

// Servo command in degrees for a wheel angle in radians, clamped to the range
// main proved. Nothing in the firmware commands a wheel angle any more, so this
// stays as the one conversion for anything that does.
inline float servoDegFromWheel(float wheelRad) {
  const float deg =
      servo::CENTRE_DEG + servo::DIR * servo::PER_WHEEL * wheelRad * 57.29578f;
  if (deg < servo::MIN_DEG) return servo::MIN_DEG;
  if (deg > servo::MAX_DEG) return servo::MAX_DEG;
  return deg;
}
// end

struct Config {
  // Sensor offsets in metres. Unmeasured, so zero, and the calibration folds
  // them into S0 and the trim, which is what replaces them.
  float e_L = 0.00f;
  float e_R = 0.00f;

  // Run-time calibration. S0 is the sensor sum with the car squared up, and the
  // trim is half the centred difference. Averaging keeps noise out of the run,
  // and the batch is retaken on every start so the pose the run begins from is
  // the pose it is measured against. 32 ticks is 1.1 s at the control period,
  // and the cap is the backstop for a car that cannot see both walls.
  float S0            = 0.0f;
  float trim          = 0.0f;
  int   calib_samples = 32;
  uint32_t calib_timeout_ms = 5000;

  // Provisional, this tree's figure for the 468 mm car, and V0 C1 replaces it
  // with a timed 20 cm reverse at the chosen reverse duty. The firmware no
  // longer steers from it, so it only dead-reckons the travel watchdog.
  float speed = 0.15f;

  // Duty on the drive legs. 55 is main's reverse cruise and 10 its crawl, which
  // is what it slowed to inside 160 mm of the wall. 90 is main's forward
  // adjustment. All three are replaced by the duty that produces the measured
  // speed, V0 C1 reverse and V0 C2 forward.
  uint8_t reverse_duty  = 55;
  uint8_t crawl_duty    = 10;
  uint8_t stroke_duty   = 90;
  float   crawl_below_m = 0.160f;

  // Braking distance, that is the roll-on after cutting drive. V0 C3 measures
  // it and the value is held until V5.
  float d_stop = 0.020f;

  // Tolerances, bay target and the rear trigger. The bay is 300 wide and 450
  // deep, so 90 mm a side and 216 mm longer than the car. The tolerances are
  // reporting only now, and the rear trigger is what ends the reverse.
  // rear_target is a reading, and the rear sensor face is the rearmost point of
  // the car, 35.5 mm behind the rear axle when taped on 2026-11-02, so the
  // reading is the tail clearance itself. 0.038 with the 0.012 margin asks for
  // 0.050 and leaves the tail 50 mm off the wall, 166 mm inside the mouth.
  float tol_off     = 0.010f;
  float tol_align   = 0.061f;
  float rear_target = 0.038f;
  float rear_margin = 0.012f;

  // The maneuver, all fixed times. The car reverses straight, then runs one
  // forward pulse at a fixed lock and one reverse pulse at the opposite lock,
  // and repeats that until corrections_max corrections are done or the car is
  // at the wall. Nothing here is computed from a measurement, because a
  // computed profile needs a measured car and this one is not measured.
  int   corrections_max = 3;
  float pull_angle_deg  = 25.0f;
  // The one polarity to flip if the car corrects the wrong way on the car. The
  // direction itself comes from the sign of gL - gR, the way main did it, and
  // the linkage sign has never been measured, V0 B2.
  float pull_steer_sign = 1.0f;
  // main ended its forward leg when the rear reading passed 22 cm, and that is
  // the only measurement that can shorten a pull.
  float pull_rear_exit_m = 0.220f;
  uint32_t pull_forward_ms = 300;
  uint32_t pull_reverse_ms = 300;
  uint32_t settle_ms       = 150;

  // Watchdogs. Rear readings in a row, the furthest reverse with no odometer,
  // and a wall clock on the reverse leg so a dead sensor cannot reverse the car
  // off the far end of the bay.
  int   rear_fault_ticks   = 3;
  float back_in_max_travel = 2.5f;
  uint32_t back_in_max_ms  = 10000;

  // The control period the travel dead reckoning integrates over.
  float dt = 0.033f;
};

// Board pin map for the ESP32-S3 Super Mini. GPIO 26 to 32 carry the
// in-package flash, 19 and 20 are USB, 43 and 44 are UART0, and 0, 3, 45 and 46
// are strapping pins, so none of them appear here. Ten signals are needed and
// only seven pins are free of any boot role. The other three, SCL, the rear
// XSHUT and the servo, sit on FSPI IO MUX pins, which are free only because
// nothing here drives SPI2.
namespace board {
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr int PIN_XSHUT_L = 1;
constexpr int PIN_XSHUT_R = 2;
constexpr int PIN_XSHUT_REAR = 10;
constexpr int PIN_SERVO = 11;
// Channel A on 4 and 5 is the left motor, B on 6 and 7 the right. Both pairs
// are provisional and V0 C5 decides which channel drives which side.
constexpr int PIN_MOTOR_IN1 = 4;
constexpr int PIN_MOTOR_IN2 = 5;
constexpr int PIN_MOTOR_B1 = 6;
constexpr int PIN_MOTOR_B2 = 7;

// The two motors are mirrored, so one channel has to drive flipped or the car
// spins on the spot. V0 C4 settles both with a forward pulse, wheels off the
// ground.
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

// The address each sensor ends up on. 0x29 is where they all start.
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;

// 30 Hz, so a measurement every 33 ms, with the budget to match. Longer hands
// the loop the same sample twice, shorter races the sensor's own sequence.
constexpr uint32_t SENSOR_BUDGET_US = 33000;
constexpr uint32_t SENSOR_PERIOD_MS = 33;

// The driver's default timeout is none, which blocks on a dead sensor. 50 ms
// outlasts one measurement, so a healthy sensor never trips it.
constexpr uint16_t SENSOR_TIMEOUT_MS = 50;
}
// end

// Ten signals on one board, so two functions sharing a pin is a harness mistake
// that would only show up as two peripherals fighting. The build catches it.
constexpr bool pinsDistinct() {
  const int p[] = {board::PIN_I2C_SDA, board::PIN_I2C_SCL, board::PIN_XSHUT_L,
                   board::PIN_XSHUT_R, board::PIN_XSHUT_REAR, board::PIN_SERVO,
                   board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                   board::PIN_MOTOR_B1, board::PIN_MOTOR_B2};
  for (int i = 0; i < 10; ++i)
    for (int j = i + 1; j < 10; ++j)
      if (p[i] == p[j]) return false;
  return true;
}
// end
static_assert(pinsDistinct(), "board pins must be unique");

namespace timing {
// 30 Hz, which is what Config::dt says. The maneuver is timed from the clock,
// and the travel dead reckoning assumes this period.
constexpr uint32_t CONTROL_PERIOD_MS = 33;

// Ten a second is enough to watch a maneuver that takes seconds, and keeps the
// socket work far below the tick rate.
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

// How long the web task waits for the configured network before falling back
// to its own access point.
constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
}
// end

// The travel dead reckoning integrates over one control period, so a silent
// disagreement with Config::dt is a real bug and not a tuning choice.
static_assert(Config{}.dt >= 0.001f * (float)timing::CONTROL_PERIOD_MS - 0.0005f &&
              Config{}.dt <= 0.001f * (float)timing::CONTROL_PERIOD_MS + 0.0005f,
              "Config::dt must agree with CONTROL_PERIOD_MS to the millisecond");
