#pragma once

#include <Arduino.h>

// Everything the firmware needs to know about this car in one file. Pins,
// the three sensor addresses, the measured chassis and sensor stations, the
// servo range, the gains, the duties and the times all live here. A value that
// is still a guess says so in its comment.

// Board pins for the ESP32-S3 Super Mini. GPIO 26 to 32 carry the in-package
// flash, 19 and 20 are USB, 43 and 44 are UART0, and 0, 3, 45 and 46 are
// strapping pins, so none of them appear here. The rear XSHUT line, the SCL
// line and the servo sit on FSPI IO MUX pins, which are free because nothing
// here drives SPI2.
namespace board {
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr int PIN_XSHUT_L = 1;
constexpr int PIN_XSHUT_R = 2;
constexpr int PIN_XSHUT_REAR = 10;
constexpr int PIN_SERVO = 11;

// Channel A on 4 and 5 is the left motor, B on 6 and 7 the right. Both pairs
// are provisional and a forward pulse with the wheels off the ground settles
// which channel drives which side.
constexpr int PIN_MOTOR_IN1 = 4;
constexpr int PIN_MOTOR_IN2 = 5;
constexpr int PIN_MOTOR_B1 = 6;
constexpr int PIN_MOTOR_B2 = 7;

// The two motors are mirrored, so one channel has to drive flipped or the car
// spins on the spot. Both flags are from main and are unobserved on this car.
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Every VL53L0X powers up at 0x29. They are moved apart during bring-up, and
// the left sensor is the one that keeps 0x29.
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;
}

// The measured chassis and sensor stations, taken from the built car on
// 2026-10-29 and 2026-11-02. Nothing in this firmware derives a pose, so these
// are recorded rather than used. They are kept because the next feature that
// needs a heading term has to start from the measurements, not from a guess.
namespace car {
constexpr float L_wb = 0.183f;    // wheelbase, metres
constexpr float t_front = 0.0905f; // front kingpin bore spacing, metres
constexpr float L_car = 0.234f;   // body length, metres
constexpr float d_L = 0.067f;     // left sensor face ahead of the rear axle
constexpr float d_R = -0.147f;    // right sensor face, negated station
}

// The steering servo. CENTRE_DEG is the command that should point the wheels
// straight, and INVERTED flips the sign between a positive lateral error and
// the servo command. Both are main's values and are unmeasured on this car.
// attach(PIN_SERVO) uses the library default pulse map, 544 to 2400 us over 0
// to 180 degrees, so 90 degrees is about 1472 us. Whole degrees are written and
// every command is clamped to 45 to 135, which is the range main drove.
namespace servo {
constexpr int CENTRE_DEG = 90;
constexpr int MIN_DEG = 45;
constexpr int MAX_DEG = 135;
constexpr bool INVERTED = false;
}

// One measurement every 33 ms, with a 50 ms timeout so a dead part returns
// instead of blocking the loop. A raw reading of 0 or anything over 8000 mm is
// rejected. The library reports 8190 mm with no target and 65535 mm on an I2C
// timeout, so both fall outside the accepted range. The EMA is main's.
namespace sensor {
constexpr uint32_t BUDGET_US = 33000;
constexpr uint32_t PERIOD_MS = 33;
constexpr uint16_t TIMEOUT_MS = 50;
constexpr uint16_t MAX_VALID_MM = 8000;
constexpr float EMA_ALPHA = 0.35f;
}

// main's PID on the lateral error. The error is in centimetres and the output
// is a servo offset in degrees, so the gains are main's numbers unchanged. The
// deadband is 8 mm, which is main's 0.8 cm. The integral clamp and the
// derivative low pass are main's too. None of it has been tuned on this car.
namespace pid {
constexpr float KP = 1.8f;
constexpr float KI = 0.04f;
constexpr float KD = 0.40f;
constexpr float INTEGRAL_CLAMP = 25.0f;
constexpr float DERIVATIVE_ALPHA = 0.6f;
constexpr float DEADBAND_CM = 0.8f;
}

// The maneuver thresholds in millimetres. The rear sensor face is the rearmost
// point of the car, so its reading is the tail clearance itself. The 76 mm stop
// leaves the tail about 50 mm off the wall once the car rolls on.
namespace parking {
constexpr float REAR_STOP_MM = 76.0f;
constexpr float PULL_GAP_MM = 12.0f;
constexpr float PULL_REAR_MAX_MM = 130.0f;
constexpr float PULL_EXIT_MM = 220.0f;
constexpr float PULL_STEER_DEG = 25.0f;
constexpr unsigned long PULL_FORWARD_MS = 1400;
constexpr unsigned long PAUSE_MS = 150;
constexpr unsigned long REVERSE_MAX_MS = 4000;
constexpr int MAX_PULLS = 3;
}

// Duty on the two drive legs. 200 is the only duty ever observed to move this
// car on the DRV8833 at 5 V, and it did so forward on the bench on 2026-11-02.
// main's 55, 10 and 90 were an L298N at a different rail. 20 kHz is above
// hearing, which is what the owner asked for after the default 1 kHz whine.
// The DRV8833 datasheet sets no maximum PWM input frequency and its 450 ns
// input deglitch time is a hundred times shorter than a 50 us period, so the
// input follows the signal. Its switching loss is 10 to 30 percent of the DC
// loss and grows with frequency, which is the cost of the quiet band. Pololu's
// own advice is to use an ultrasonic frequency such as 20 kHz for motor quiet,
// and it is their DRV8833 board this is.
namespace drive {
constexpr uint8_t DUTY_REVERSE = 200;
constexpr uint8_t DUTY_FORWARD = 200;
constexpr uint32_t PWM_HZ = 20000;
constexpr uint8_t PWM_BITS = 8;
}

namespace timing {
constexpr unsigned long LOOP_MS = 25;
constexpr unsigned long LOG_MS = 100;
constexpr unsigned long WIFI_TIMEOUT_MS = 15000;
constexpr unsigned long SENSOR_SETTLE_MS = 120;
}
