// Bench copy of the drive path, and nothing else. It is a separate PlatformIO
// project under bench/, so the parking firmware and its environment are
// untouched. No sensors, no servo, no radio: the car moves the moment the
// board is powered, and it keeps moving until the power is cut, so it can be
// tested with no console attached.

#include <Arduino.h>

#include "config.h"

// High enough that a healthy car always moves on the 5 V rail. If the car pulls
// away backwards, the inversion flag for that side is wrong, which is what V0
// C4 settles.
constexpr uint8_t DRIVE_DUTY = 200;

// One DRV8833 channel, driven exactly as src/actuators.h drives it. The copy is
// deliberate: this build must not pull the sensor stack in, and the pin and
// inversion constants still come from ../src/config.h, so a rewiring propagates
// here on the next build. Only the lines below can drift.
static void driveChannel(int inA, int inB, int dir, uint8_t duty, bool invert) {
  // Every write is analogWrite, including the zero side, because the first PWM
  // on a pin hands it to the LEDC peripheral and digitalWrite then refuses it.
  if (dir == 0) {
    analogWrite(inA, 0);
    analogWrite(inB, 0);
    return;
  }
  const bool fwd = (dir > 0) != invert;
  if (fwd) {
    analogWrite(inA, duty);
    analogWrite(inB, 0);
  } else {
    analogWrite(inA, 0);
    analogWrite(inB, duty);
  }
}

void setup() {
  // The driver pins go low first, exactly as the parking firmware does, so the
  // car cannot creep while the board is starting.
  const int pins[] = {pins::MOTOR_L_IN1, pins::MOTOR_L_IN2,
                      pins::MOTOR_R_IN1, pins::MOTOR_R_IN2};
  for (int p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.begin(115200);
  Serial.printf("bench drive, forward at %u, power off to stop\n",
                (unsigned)DRIVE_DUTY);

  driveChannel(pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, +1, DRIVE_DUTY,
               pins::INVERT_LEFT_MOTOR);
  driveChannel(pins::MOTOR_R_IN1, pins::MOTOR_R_IN2, +1, DRIVE_DUTY,
               pins::INVERT_RIGHT_MOTOR);
}

void loop() {
  // Nothing to do. The drive stays on until the board loses power.
  delay(1000);
}
