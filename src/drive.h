#pragma once

#include <Arduino.h>

#include "config.h"

// The DRV8833 on four pins, one channel per side, on the LEDC peripheral at
// 20 kHz and 8 bits. See drive::PWM_HZ in config.h for why 20 kHz.
//
// Driving is fast decay: the PWM goes to one input and the other stays low, so
// the winding alternates between drive and coast. The stop is a brake, which on
// this part is both inputs high and the winding shorted. Both inputs low is
// coast, not brake, and is never used here. A brake is what keeps the car from
// rolling after the rear limit and while parked.

// The four inputs are LEDC channels at 20 kHz and 8 bits, which is all this
// part needs, two plain logic inputs per channel and no gate drive. MCPWM buys
// nothing here, its dead time, complementary outputs and fault inputs are for
// an H bridge made of FETs, and the Arduino core 3.3.12 has no MCPWM wrapper at
// all. The servo is the other way round, ESP32Servo 3.2.1 puts it on MCPWM at
// 50 Hz on the S3, so the servo and the motors never share a timer.
// end

class Drive {
public:
  void begin();
  void forward(uint8_t duty);
  void reverse(uint8_t duty);
  void brake();

private:
  static void channel(int inA, int inB, int dir, uint8_t duty, bool invert);
};

inline void Drive::begin() {
  const int pins[] = {board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                      board::PIN_MOTOR_B1, board::PIN_MOTOR_B2};
  for (int p : pins) {
    if (!ledcAttach(p, drive::PWM_HZ, drive::PWM_BITS)) {
      Serial.printf("ledc attach failed on gpio %d\n", p);
    }
    ledcWrite(p, 0);
  }
  brake();
}

inline void Drive::forward(uint8_t duty) {
  channel(board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2, +1, duty,
          board::INVERT_LEFT_MOTOR);
  channel(board::PIN_MOTOR_B1, board::PIN_MOTOR_B2, +1, duty,
          board::INVERT_RIGHT_MOTOR);
}

inline void Drive::reverse(uint8_t duty) {
  channel(board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2, -1, duty,
          board::INVERT_LEFT_MOTOR);
  channel(board::PIN_MOTOR_B1, board::PIN_MOTOR_B2, -1, duty,
          board::INVERT_RIGHT_MOTOR);
}

// Both inputs high on both channels. The DRV8833 holds the outputs low, so the
// windings are shorted and the car is held rather than left to coast.
inline void Drive::brake() {
  ledcWrite(board::PIN_MOTOR_IN1, 255);
  ledcWrite(board::PIN_MOTOR_IN2, 255);
  ledcWrite(board::PIN_MOTOR_B1, 255);
  ledcWrite(board::PIN_MOTOR_B2, 255);
}

// One channel in fast decay, PWM on the driven input and the other low. The
// inversion flag is the whole difference between the mirrored motors.
inline void Drive::channel(int inA, int inB, int dir, uint8_t duty,
                           bool invert) {
  const bool forward = (dir > 0) != invert;
  if (forward) {
    ledcWrite(inA, duty);
    ledcWrite(inB, 0);
  } else {
    ledcWrite(inA, 0);
    ledcWrite(inB, duty);
  }
}
