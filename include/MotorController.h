#pragma once

#include <Arduino.h>

#include "ParkingConfig.h"

// The DRV8833 on four pins, one channel per side. The PWM runs on the LEDC
// peripheral at 20 kHz and 8 bits rather than the analogWrite default of 1 kHz,
// because 1 kHz is an audible whine. See drive::PWM_HZ in ParkingConfig.h for
// why 20 kHz is safe for this driver.
//
// Both inputs low is coast on the DRV8833, so stop means the motor turns off
// rather than brakes. Only the driven input carries the duty and the other one
// stays at zero, which is fast decay on this part.

enum MotorDirection {
  MOTOR_STOPPED = 0,
  MOTOR_REVERSE,
  MOTOR_FORWARD
};

class MotorController {
public:
  void begin();
  void forward(uint8_t duty);
  void reverse(uint8_t duty);
  void stop();

  MotorDirection getDirection() const { return direction_; }
  uint8_t getSpeed() const { return speed_; }
  const char* getDirectionString() const;

private:
  uint8_t speed_ = 0;
  MotorDirection direction_ = MOTOR_STOPPED;

  static void driveChannel(int inA, int inB, int dir, uint8_t duty,
                           bool invert);
};

inline void MotorController::begin() {
  const int pins[] = {board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                      board::PIN_MOTOR_B1, board::PIN_MOTOR_B2};
  for (int p : pins) {
    if (!ledcAttach(p, drive::PWM_HZ, drive::PWM_BITS)) {
      Serial.printf("ledc attach failed on gpio %d\n", p);
    }
    ledcWrite(p, 0);
  }
  stop();
}

inline void MotorController::forward(uint8_t duty) {
  speed_ = duty;
  direction_ = MOTOR_FORWARD;
  driveChannel(board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2, +1, duty,
               board::INVERT_LEFT_MOTOR);
  driveChannel(board::PIN_MOTOR_B1, board::PIN_MOTOR_B2, +1, duty,
               board::INVERT_RIGHT_MOTOR);
}

inline void MotorController::reverse(uint8_t duty) {
  speed_ = duty;
  direction_ = MOTOR_REVERSE;
  driveChannel(board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2, -1, duty,
               board::INVERT_LEFT_MOTOR);
  driveChannel(board::PIN_MOTOR_B1, board::PIN_MOTOR_B2, -1, duty,
               board::INVERT_RIGHT_MOTOR);
}

inline void MotorController::stop() {
  speed_ = 0;
  direction_ = MOTOR_STOPPED;
  ledcWrite(board::PIN_MOTOR_IN1, 0);
  ledcWrite(board::PIN_MOTOR_IN2, 0);
  ledcWrite(board::PIN_MOTOR_B1, 0);
  ledcWrite(board::PIN_MOTOR_B2, 0);
}

inline const char* MotorController::getDirectionString() const {
  switch (direction_) {
  case MOTOR_REVERSE:
    return "REV";
  case MOTOR_FORWARD:
    return "FWD";
  default:
    return "STOP";
  }
}

// One DRV8833 channel. A direction of zero is the coast the rear limit wants,
// with both inputs low.
inline void MotorController::driveChannel(int inA, int inB, int dir,
                                          uint8_t duty, bool invert) {
  // A zero direction is the coast the rear limit wants, both inputs low ---
  if (dir == 0) {
    ledcWrite(inA, 0);
    ledcWrite(inB, 0);
    return;
  }
  // end

  // The inversion flag is the whole difference between the mirrored motors ---
  const bool forward = (dir > 0) != invert;
  if (forward) {
    ledcWrite(inA, duty);
    ledcWrite(inB, 0);
  } else {
    ledcWrite(inA, 0);
    ledcWrite(inB, duty);
  }
  // end
}
