// Servo test for the bench. It drives the steering servo from the LEDC
// peripheral directly, so the bench project stays free of libraries, and it
// sweeps the command from one end of the range to the other and back until the
// power is cut. Power the car, watch the wheels turn, power it down.
//
// The signal is the parking image's pulse map, 544 to 2400 us over 0 to 180
// degrees, which is what plain attach() gives, so a pulse printed here means
// the same thing on the car. The command never claims a direction: the linkage
// is mirrored in a way nobody has measured, so only the sweep direction is
// printed and the wheels say the rest.

#include <Arduino.h>

#include "config.h"

constexpr int PIN = board::PIN_SERVO;
constexpr uint32_t LEDC_HZ = 50;
constexpr uint8_t LEDC_BITS = 16;
constexpr uint32_t DWELL_MS = 400;
constexpr int CENTRE_DEG = 90;
constexpr int STEP_DEG = 5;

// main's clamped range, the 45 to 135 degrees its servo command never left.
// The parking loop asks up to 151, so this range is not the full travel. Widen
// either end here once the knuckles are known to clear their stops.
constexpr int LO_DEG = 45;
constexpr int HI_DEG = 135;

// The sweep position and its direction, carried between calls. A positive
// direction is the increasing half of the sweep.
static int sweep_deg = CENTRE_DEG;
static int sweep_dir = 1;

// 544 to 2400 us over 0 to 180 degrees, the library default the parking image
// attaches with.
static uint32_t pulseUs(int deg) { return 544UL + (uint32_t)deg * 1856UL / 180UL; }

// One 50 Hz period is 20000 us, so the duty fraction is the pulse over the
// period at the 16 bit resolution, and the half count rounds the division.
static void command(int deg) {
  const uint32_t us = pulseUs(deg);
  const uint32_t duty = (uint32_t)((float)us * 65536.0f / 20000.0f + 0.5f);
  ledcWrite(PIN, duty);
}

// Report the step, write it, then hold it for the dwell so the wheels have
// time to reach the command before the next one.
static void step(int deg, bool increasing) {
  const uint32_t us = pulseUs(deg);
  Serial.printf("angle %3d deg, pulse %4lu us, %s\n", deg, (unsigned long)us,
                increasing ? "increasing" : "decreasing");
  command(deg);
  delay(DWELL_MS);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  ledcAttach(PIN, LEDC_HZ, LEDC_BITS);
  Serial.printf("servo test, pin %d, %d to %d degrees, %lu to %lu us\n", PIN,
                LO_DEG, HI_DEG, (unsigned long)pulseUs(LO_DEG),
                (unsigned long)pulseUs(HI_DEG));

  // Centre first and hold for a second, so the wheels can be checked against
  // the straight-ahead mark before the sweep starts.
  command(CENTRE_DEG);
  delay(1000);
  // end
}

void loop() {
  // Advance one step and bounce at each end. The end is tested before the step
  // is taken, so the printed angle never leaves the two limits.
  int next = sweep_deg + sweep_dir * STEP_DEG;
  if (next > HI_DEG || next < LO_DEG) {
    sweep_dir = -sweep_dir;
    next = sweep_deg + sweep_dir * STEP_DEG;
  }
  sweep_deg = next;
  step(sweep_deg, sweep_dir > 0);
  // end
}
