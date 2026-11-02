#pragma once

#include <cstdint>

// Every tunable for the car, and the one file to edit after measuring. The
// maneuver is fixed time and fixed duty, so there is no pose and no speed
// here. What moved the car on the bench is what this file sends. The order
// runs the steering range, the maneuver, the board pins and the task timing.

// The servo range main drove and clamped to, 90 centre and 45 to 135 either
// side. The linkage is only assumed, so every write is held inside it rather
// than at the servo's own 0 to 180.
namespace servo {
constexpr float CENTRE_DEG = 90.0f;
constexpr float MIN_DEG    = 45.0f;
constexpr float MAX_DEG    = 135.0f;
}
// end

// A reading is used only when it looks like a real gap. The VL53L0X reports
// 8.19 m with no target and 65.535 m on a timeout, so both fall outside and are
// treated as no reading rather than clamped. This is the one sensor rule the
// maneuver and the telemetry share.
inline bool validReading(float m) { return m > 0.0f && m < 8.0f; }
// end

// The rear sensor is the one that sees the wall. A reading of exactly 0 is the
// VL53L0X minimum-range status, which main read as 3 cm and treated as valid at
// the wall, so 0 counts here too. Without this rule the sensor fails at the
// wall and the reverse leg gives up a few centimetres from where it should stop.
inline bool rearValidReading(float m) { return m >= 0.0f && m < 8.0f; }
// end

struct Config {
  // Duty on the drive legs. 200 is the one duty ever observed to move this car,
  // on the bench image, and main's 55, 10 and 90 came from a different driver on
  // different pins. One number to change later.
  uint8_t reverse_duty = 200;
  uint8_t stroke_duty  = 200;

  // Braking distance, that is the roll-on after cutting drive. V0 C3 measures
  // it, and the value is held at 20 mm until then.
  float d_stop = 0.020f;

  // The rear trigger. rear_target is the wanted gap, 38 mm, and the sensor face
  // is the rearmost point of the car, so the reading is the gap to the wall. The
  // trigger adds the roll-on and a 12 mm latency margin, which asks the motor to
  // stop at 70 mm.
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
  uint32_t pull_forward_ms = 500;
  uint32_t pull_reverse_ms = 500;

  // One reverse leg cap. The leg normally ends on the rear sensor, and this is
  // only the stop for a car that never reaches the wall or a sensor that never
  // answers.
  uint32_t back_in_max_ms = 4000;
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
// Channel A on 4 and 5 is the left motor, B on 6 and 7 the right.
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
// 30 Hz, which is the sensor period. The maneuver is timed from the clock.
constexpr uint32_t CONTROL_PERIOD_MS = 33;

// Ten a second is enough to watch a maneuver that takes seconds, and keeps the
// socket work far below the tick rate.
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

// How long the web task waits for the configured network before falling back
// to its own access point.
constexpr uint32_t WIFI_TIMEOUT_MS = 15000;
}
// end
