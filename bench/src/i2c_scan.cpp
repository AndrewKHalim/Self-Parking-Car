// I2C presence test. It reports what the two bus lines are actually doing,
// then scans with the pins as wired and again with SDA and SCL swapped, so the
// console answers three questions at once: are the lines pulled up, is anything
// on the bus, and is it wired the right way round. No sensor library, because a
// scan only needs Wire and the addresses.
//
// The idle voltage is read on the ADC, since GPIO 8 and 9 are ADC1 channels on
// the S3. A line clamped near 0 mV is held low, a few hundred is floating with
// no pull-up, and 3300 or more is a live pull-up, 5 V included.
//
// The three XSHUT lines are driven high first, since a VL53L0X held in reset
// never answers and looks exactly like a broken bus.

#include <Arduino.h>
#include <Wire.h>

#include "ParkingConfig.h"

// The sensors all power up at 0x29, so a presence scan shows one entry there
// whatever is fitted. That is what this test needs. Addressing them apart is
// the firmware's job, not this one's.
static void scan(TwoWire& bus, int sda, int scl, const char* label) {
  Serial.printf("--- %s: SDA %d, SCL %d\n", label, sda, scl);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.printf("    device at 0x%02X\n", a);
      found++;
    }
  }
  Serial.printf("    %d device(s)\n", found);
}

// What the line does with nothing driving it. Floating means no pull-up is
// fitted or the wire is not connected, and a clamped low line means a short or
// a device holding the bus down.
static void levels(const char* label) {
  Serial.printf("--- line levels, %s\n", label);
  const int pins[] = {board::PIN_I2C_SDA, board::PIN_I2C_SCL};
  for (int p : pins) {
    pinMode(p, INPUT);
    delay(5);
    const int mvFloat = analogReadMilliVolts(p);
    const int readFloat = digitalRead(p);
    pinMode(p, INPUT_PULLUP);
    delay(5);
    const int mvPull = analogReadMilliVolts(p);
    const int readPull = digitalRead(p);
    Serial.printf("    gpio %d: %4d mV free (%d), %4d mV pulled up (%d)\n", p,
                  mvFloat, readFloat, mvPull, readPull);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Out of reset, or nothing answers.
  pinMode(board::PIN_XSHUT_L, OUTPUT);
  pinMode(board::PIN_XSHUT_R, OUTPUT);
  pinMode(board::PIN_XSHUT_REAR, OUTPUT);
  digitalWrite(board::PIN_XSHUT_L, HIGH);
  digitalWrite(board::PIN_XSHUT_R, HIGH);
  digitalWrite(board::PIN_XSHUT_REAR, HIGH);
  delay(50);

  // Read back what the pads are doing, so a shorted XSHUT shows up as a pin
  // driven high that reads low.
  const int shuts[] = {board::PIN_XSHUT_L, board::PIN_XSHUT_R,
                       board::PIN_XSHUT_REAR};
  for (int p : shuts) {
    Serial.printf("--- xshut gpio %d reads %d when driven high\n", p,
                  digitalRead(p));
  }

  levels("before the bus is opened");

  const bool up = Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);
  Serial.printf("wire begin %s\n", up ? "ok" : "FAILED");
  scan(Wire, board::PIN_I2C_SDA, board::PIN_I2C_SCL, "as wired");
  Wire.end();
  delay(10);

  levels("after the bus is closed");

  const bool swapped = Wire.begin(board::PIN_I2C_SCL, board::PIN_I2C_SDA);
  Serial.printf("wire begin swapped %s\n", swapped ? "ok" : "FAILED");
  scan(Wire, board::PIN_I2C_SCL, board::PIN_I2C_SDA, "swapped");
  Wire.end();

  Serial.println("scan done");
}

void loop() {
  // One shot, so the result stays on screen.
  delay(1000);
}
