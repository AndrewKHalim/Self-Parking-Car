// Servo sweep for measuring the steering linkage. It drives the servo signal
// with the LEDC peripheral directly, so the bench project stays free of
// libraries, and steps the command through the range the firmware uses,
// printing each angle and the pulse it sends.
//
// The firmware writes whole degrees with servo::CENTRE_DEG and servo::DIR, and
// both are unmeasured. This sweep is how they are measured: watch where the
// wheels come straight, which is the centre, and which way they turn, which is
// the sign for DIR. Widen the range only after checking the knuckles clear
// their stops.
//
// The pulse map is the one the firmware attaches with, 544 to 2400 us over 0 to
// 180 degrees, so a pulse printed here means the same thing on the car.

#include <Arduino.h>

#include "ParkingConfig.h"

constexpr int PIN = board::PIN_SERVO;
constexpr uint32_t LEDC_HZ = 50;
constexpr uint8_t LEDC_BITS = 16;
constexpr uint32_t DWELL_MS = 1000;

// The firmware's clamped range, 45 to 135 degrees. If 135 turns out to be the
// mechanical end, narrow these before widening them.
constexpr int LO_DEG = servo::MIN_DEG;
constexpr int HI_DEG = servo::MAX_DEG;
constexpr int CENTRE_DEG = servo::CENTRE_DEG;
constexpr int STEP_DEG = 5;

// 544 to 2400 us over 0 to 180 degrees, the library default the firmware attaches with.
static uint32_t pulseUs(int deg) { return 544UL + (uint32_t)deg * 1856UL / 180UL; }

static void command(int deg) {
  const uint32_t us = pulseUs(deg);
  // One 50 Hz period is 20000 us, so the duty fraction is us/20000 of 2^16.
  const uint32_t duty = (uint32_t)((float)us * 65536.0f / 20000.0f + 0.5f);
  ledcWrite(PIN, duty);
  // end
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
  command(CENTRE_DEG);
  for (int deg = CENTRE_DEG + STEP_DEG; deg <= HI_DEG; deg += STEP_DEG)
    command(deg);
  command(CENTRE_DEG);
  for (int deg = CENTRE_DEG - STEP_DEG; deg >= LO_DEG; deg -= STEP_DEG)
    command(deg);
  command(CENTRE_DEG);

  Serial.println("done, power cycle to run again");
}

void loop() {
  // One shot, so the servo never hunts on the bench once the sweep is over.
  delay(1000);
}
