#pragma once

#include <Arduino.h>

// Everything this car needs to know about itself, in one file. Pins, the three
// sensor addresses, the measured chassis and sensor stations, the servo range,
// the sensor timing and the maneuver thresholds all live here. A value that is
// still a guess says so.
//
// The chassis and sensor stations are measured. They are recorded, not used,
// because this firmware does not compute a heading. They stay because the pair of
// side faces cannot separate lateral offset from yaw on their own, and any
// later attempt has to start from these numbers.

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

namespace car {
constexpr float L_wb = 0.183f;
constexpr float t_front = 0.0905f;
constexpr float L_car = 0.234f;
constexpr float d_L = 0.067f;
constexpr float d_R = -0.147f;
// The two side faces measured 95 mm apart on the built mounts.
constexpr float FACE_SEPARATION_M = 0.095f;
}

// The steering servo is never commanded away from centre in this firmware. The
// range and the centre are kept so the bench sweep and any later code measure
// the same numbers. attach(PIN_SERVO) uses the library default pulse map, 544
// to 2400 us over 0 to 180 degrees, so 90 degrees is about 1472 us.
namespace servo {
constexpr int CENTRE_DEG = 90;
constexpr int MIN_DEG = 45;
constexpr int MAX_DEG = 135;
}

// One measurement every 33 ms with a 50 ms I2C timeout. The library reports
// 8190 mm for no target and 65535 mm when a read times out. A raw reading of 0
// to 29 mm is the part's minimum range, so it means the face is at the wall and
// is not a distance to trust as a number.
namespace sensor {
constexpr uint32_t BUDGET_US = 33000;
constexpr uint32_t PERIOD_MS = 33;
constexpr uint16_t TIMEOUT_MS = 50;
constexpr uint16_t NO_TARGET_MM = 8190;
constexpr uint16_t TIMEOUT_MM = 65535;
constexpr uint16_t WALL_MAX_MM = 29;
constexpr uint16_t REAR_VALID_MIN_MM = 30;
constexpr uint16_t REAR_VALID_MAX_MM = 800;
constexpr uint16_t SIDE_GUARD_MM = 25;
}

namespace parking {
constexpr float REAR_STOP_MM = 90.0f;
constexpr unsigned long CHECK_FRAMES = 5;
constexpr unsigned long REVERSE_MAX_MS = 3000;
constexpr unsigned long BRAKE_MS = 300;
constexpr int REAR_INVALID_LIMIT = 2;
}

// Duty 200 is the only duty ever observed to move this car on the DRV8833 at
// 5 V, and it did so forward on the bench on 2026-11-02. 20 kHz is above
// hearing, which is what the owner asked for after the default 1 kHz whine.
//
// The TI DRV8833 datasheet, SLVSAR1E, sets no maximum PWM input frequency and
// gives a 450 ns input deglitch time, a hundred times shorter than a 50 us
// period, so the input follows the signal. Switching loss is 10 to 30 percent
// of the DC loss and grows with frequency, which is the cost of the quiet band.
// Pololu's forum answer is to use an ultrasonic frequency such as 20 kHz to
// remove motor whine (PWM Frequency and Noise, thread 4636, 2012), and this car
// runs Pololu's DRV8833 carrier.
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
constexpr unsigned long SENSOR_SETTLE_MS = 150;
}
