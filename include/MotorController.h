#pragma once

#include <Arduino.h>

#include "ParkingConfig.h"

// The DRV8833 on four pins, one channel per side. Every write is analogWrite,
// including the zero leg, because the first PWM on a pin hands it to the LEDC
// peripheral and a later digitalWrite does nothing. Stop therefore writes zero
// to all four pins and puts both channels in the gentle brake.

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
  pinMode(board::PIN_MOTOR_IN1, OUTPUT);
  pinMode(board::PIN_MOTOR_IN2, OUTPUT);
  pinMode(board::PIN_MOTOR_B1, OUTPUT);
  pinMode(board::PIN_MOTOR_B2, OUTPUT);
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
  analogWrite(board::PIN_MOTOR_IN1, 0);
  analogWrite(board::PIN_MOTOR_IN2, 0);
  analogWrite(board::PIN_MOTOR_B1, 0);
  analogWrite(board::PIN_MOTOR_B2, 0);
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

// One DRV8833 channel.
inline void MotorController::driveChannel(int inA, int inB, int dir,
                                          uint8_t duty, bool invert) {
  // A direction of zero is the brake the rear limit wants, with both inputs
  // low.
  if (dir == 0) {
    analogWrite(inA, 0);
    analogWrite(inB, 0);
    return;
  }
  // end

  // The inversion flag is the whole difference between the two mirrored
  // motors. Only the driven leg gets the duty and the other stays at zero.
  const bool forward = (dir > 0) != invert;
  if (forward) {
    analogWrite(inA, duty);
    analogWrite(inB, 0);
  } else {
    analogWrite(inA, 0);
    analogWrite(inB, duty);
  }
  // end
}
