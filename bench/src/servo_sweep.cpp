// Servo sweep for measuring the steering linkage. It drives the servo signal
// with the LEDC peripheral directly, so the bench project stays free of
// libraries, and steps the command through the range main drove, printing each
// angle and the pulse it sends.
//
// The firmware scales a wheel angle by 2.9 before it writes, and that 2.9 comes
// from the CAD and not from the car. This sweep is what replaces it: watch where
// the wheels come straight, and how far they turn between the two ends.
//
// The pulse map is the one the firmware attaches with, 544 to 2400 us over 0 to
// 180 degrees, so a pulse printed here means the same thing on the car.

#include <Arduino.h>

#include "config.h"

constexpr int PIN = board::PIN_SERVO;
constexpr uint32_t LEDC_HZ = 50;
constexpr uint8_t LEDC_BITS = 16;
constexpr uint32_t DWELL_MS = 1000;

// main's clamped range, 45 to 135 degrees, which is where its servo drove
// without hitting a stop. The current loop asks up to 151, so if 135 turns out
// to be the mechanical end, delta_max and the wheel ratio both have to come
// down. Widen only after checking the knuckles clear their stops.
constexpr int LO_DEG = 45;
constexpr int HI_DEG = 135;
constexpr int STEP_DEG = 5;

// 544 to 2400 us over 0 to 180 degrees, the library default the firmware attaches with.
static uint32_t pulseUs(int deg) { return 544UL + (uint32_t)deg * 1856UL / 180UL; }

static void command(int deg) {
  const uint32_t us = pulseUs(deg);
  // One 50 Hz period is 20000 us, so the duty fraction is us/20000 of 2^16.
  const uint32_t duty = (uint32_t)((float)us * 65536.0f / 20000.0f + 0.5f);
  ledcWrite(PIN, duty);
  Serial.printf("angle %3d deg, pulse %4lu us\n", deg, (unsigned long)us);
  delay(DWELL_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ledcAttach(PIN, LEDC_HZ, LEDC_BITS);
  Serial.printf("servo sweep, pin %d, %d to %d degrees\n", PIN, LO_DEG, HI_DEG);

  // Centre first, then out to each end and back, so the straight-ahead position
  // can be read off against the steps either side of it.
  command(90);
  for (int deg = 90 + STEP_DEG; deg <= HI_DEG; deg += STEP_DEG) command(deg);
  command(90);
  for (int deg = 90 - STEP_DEG; deg >= LO_DEG; deg -= STEP_DEG) command(deg);
  command(90);

  Serial.println("done, power cycle to run again");
}

void loop() {
  // One shot, so the servo never hunts on the bench once the sweep is over.
  delay(1000);
}
