#pragma once

#include <Arduino.h>

// Everything the car needs to know about its own wiring, its own geometry and
// the thresholds the maneuver uses, in one file. A measured value carries the
// date it was taken, and a value still guessed says so. The loop and the three
// helpers read from here and never carry a second copy.

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
// spins on the spot.
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

// Every VL53L0X powers up at 0x29. They are moved apart during bring-up, and
// the left sensor is the one that keeps 0x29.
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;
}

// The side faces look straight across the car, so their readings are the gaps
// themselves and no sine factor applies. Both faces sit ahead of the rear axle,
// left 67 mm and right 147 mm. Because the stations enter the error with
// opposite signs, the gap difference is not the offset:
//     gL - gR = 2*x + 0.214*sin(psi)
// where x is the lateral offset and psi is the yaw. At 5 degrees of yaw the
// heading term alone is 18.6 mm of false error. The branch keeps main's signal,
// gL - gR, because there is no heading estimate and the gains are main's. The
// two station distances are recorded here for a future heading term; nothing in
// this branch derives a heading.
namespace geometry {
constexpr float SENSOR_AHEAD_LEFT_M = 0.067f;
constexpr float SENSOR_AHEAD_RIGHT_M = 0.147f;
}

// The steering servo. CENTRE_DEG and DIR are the two values to calibrate on the
// car: centre until the wheels point straight, then flip DIR to -1 if a
// positive lateral error steers the wrong way. The command is written as a
// whole degree on the library's default pulse map, the same way main drove this
// servo.
namespace servo {
constexpr int CENTRE_DEG = 90;
constexpr int DIR = 1;
constexpr int MIN_DEG = 45;
constexpr int MAX_DEG = 135;
}

// One measurement every 33 ms, with a 50 ms timeout so a dead sensor returns
// instead of blocking the loop. 8190 mm is the no-target reading and 65535 mm
// is the library's timeout reading; both are rejected.
namespace sensor {
constexpr uint32_t BUDGET_US = 33000;
constexpr uint32_t PERIOD_MS = 33;
constexpr uint16_t TIMEOUT_MS = 50;
constexpr uint16_t NO_TARGET_MM = 8190;
constexpr uint16_t TIMEOUT_MM = 65535;
}

// The lateral PID. The gains are main's 1.8, 0.04 and 0.40 per centimetre,
// rescaled to metres by multiplying by 100. They are inherited and unverified
// on this car. The output is degrees of servo offset, and the integral clamp
// and the derivative low pass are main's too.
namespace pid {
constexpr float KP = 180.0f;
constexpr float KI = 4.0f;
constexpr float KD = 40.0f;
constexpr float INTEGRAL_CLAMP = 25.0f;
constexpr float DERIVATIVE_ALPHA = 0.6f;
constexpr float DEADBAND_M = 0.008f;
}

// Maneuver thresholds in metres. The rear sensor face is the rearmost point of
// the car, so its reading is the tail clearance itself. Stopping the reverse at
// 70 mm leaves the tail about 50 mm off the wall once the car rolls on. The
// pull-forward gate is the crawl boundary, the last stretch where a correction
// still has room before the stop trigger.
namespace parking {
constexpr unsigned long IDLE_MS = 2000;
constexpr float REVERSE_TRIGGER_M = 0.070f;
constexpr float EMERGENCY_STOP_M = 0.040f;
constexpr float CRAWL_BELOW_M = 0.160f;
constexpr float ALIGN_TOL_M = 0.010f;
constexpr int MAX_PULL_FORWARD = 3;
constexpr unsigned long PULL_FORWARD_MS = 1400;
constexpr float PULL_FORWARD_EXIT_M = 0.220f;

// The counter-steer is main's 25 degrees, opposite the error sign, and has not
// been tuned on this car.
constexpr float PULL_FORWARD_STEER_DEG = 25.0f;
}

// Duty on the two drive legs, 0 to 255. The stock 6 V motor sees about 4.5 V
// after the driver's drop, so the reverse duty has to be high enough to break
// away from rest.
namespace drive {
constexpr uint8_t DUTY_REVERSE = 90;
constexpr uint8_t DUTY_CRAWL = 40;
constexpr uint8_t DUTY_FORWARD = 140;
}
