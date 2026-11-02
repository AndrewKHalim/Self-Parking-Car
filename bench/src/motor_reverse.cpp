// Reverse motor test for the bench. It is the forward test with the direction
// flipped, and it uses the firmware's own Drive class, so the PWM frequency,
// the duty and the inversion are exactly the ones the parking firmware sends.
//
// The car will move backwards the moment it is powered. Put the wheels off the
// ground, or leave the car clear behind it, and keep the power switch within
// reach. It drives until the power is cut.
//
// If the car drives forward instead, the inversion flags or the channel
// mapping are wrong. config.h records both as unobserved on this car, and this
// image is how they get settled.

#include <Arduino.h>

#include "../src/config.h"
#include "../src/drive.h"

Drive motors;

void setup() {
  Serial.begin(115200);
  // Wait up to two seconds for the USB console, so the banner survives a board
  // that enumerates slowly.
  // end
  const unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);
  }

  Serial.printf("bench drive, reverse at %u, pwm %lu hz, pins %d %d %d %d\n",
                (unsigned)drive::DUTY_REVERSE, (unsigned long)drive::PWM_HZ,
                board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2,
                board::PIN_MOTOR_B1, board::PIN_MOTOR_B2);

  motors.begin();
  motors.reverse(drive::DUTY_REVERSE);
}

void loop() {
  // The drive stays on until the board loses power, so the only thing left is
  // to show the board is still alive.
  // end
  Serial.printf("running, reverse at %u\n", (unsigned)drive::DUTY_REVERSE);
  delay(2000);
}
