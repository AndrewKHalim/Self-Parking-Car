// I2C presence test. It scans the bus with the pins as wired and then again
// with SDA and SCL swapped, so the console answers two questions at once: is
// anything on the bus at all, and is it wired the right way round. No sensor
// library, because a scan only needs Wire and the addresses.
//
// The three XSHUT lines are driven high first, since a VL53L0X held in reset
// never answers and looks exactly like a broken bus.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

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

void setup() {
  Serial.begin(115200);
  delay(200);

  // Out of reset, or nothing answers.
  pinMode(board::PIN_XSHUT_L, OUTPUT);
  pinMode(board::PIN_XSHUT_R, OUTPUT);
  pinMode(board::PIN_XSHUT_REAR, OUTPUT);
  digitalWrite(board::PIN_XSHUT_L, HIGH);
  digitalWrite(board::PIN_XSHUT_R, HIGH);
  digitalWrite(board::PIN_XSHUT_REAR, HIGH);
  delay(50);

  const bool up = Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);
  Serial.printf("wire begin %s\n", up ? "ok" : "FAILED");
  scan(Wire, board::PIN_I2C_SDA, board::PIN_I2C_SCL, "as wired");
  Wire.end();
  delay(10);

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
