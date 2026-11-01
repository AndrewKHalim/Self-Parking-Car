// Servo sweep for measuring the steering linkage. It drives the servo signal
// with the LEDC peripheral directly, so the bench project stays free of
// libraries, and steps the command through the range the control law uses,
// printing each angle and the pulse it sends.
//
// The point is the two numbers the firmware does not have: which command puts
// the wheels straight, and how far the wheels turn for 25 degrees of servo. The
// control law works in wheel angle and writes servo degrees as if the two were
// the same, so a crown and rack in between makes its output too small by the
// ratio.

#include <Arduino.h>

#include "config.h"

constexpr int PIN = board::PIN_SERVO;
constexpr uint32_t LEDC_HZ = 50;
constexpr uint8_t LEDC_BITS = 16;
constexpr uint32_t DWELL_MS = 1500;

// The command range the control law can produce. Widen it only after checking
// that the knuckles do not hit their stops inside it, because a servo stalled
// against a stop pulls about an amp off the shared 5 V rail.
constexpr int LO_DEG = 65;
constexpr int HI_DEG = 115;
constexpr int STEP_DEG = 5;

// 1000 to 2000 us over 0 to 180 degrees, the table the firmware attaches with.
static uint32_t pulseUs(int deg) { return 1000UL + (uint32_t)deg * 1000UL / 180UL; }

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
