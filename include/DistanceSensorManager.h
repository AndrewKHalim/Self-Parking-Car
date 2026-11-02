#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "ParkingConfig.h"

// The three VL53L0X sensors on one bus. Every part powers up at 0x29, so they
// are held in reset together and released one at a time. The rear sensor is
// released first and keeps 0x2B, then the right takes 0x2A, and the left comes
// up last and keeps 0x29. Ranging runs continuously at a 33 ms budget, so the
// loop only reads the latest result.
//
// A raw reading of 0 or anything over 8000 mm is invalid. The library returns
// 8190 mm with no target and 65535 mm when the I2C read times out, and both are
// rejected. An invalid reading leaves the last good value in place and clears
// that channel's valid flag, so a caller can tell a held value from a fresh one.
// The EMA is main's, alpha 0.35, and it snaps to the first valid reading rather
// than creeping up from zero.

struct SensorReadings {
  uint16_t rawL;
  uint16_t rawR;
  uint16_t rawRear;
  float gL;
  float gR;
  float gRear;
  bool leftValid;
  bool rightValid;
  bool rearValid;
};

class DistanceSensorManager {
public:
  bool begin();
  SensorReadings read();

private:
  VL53L0X left_;
  VL53L0X right_;
  VL53L0X rear_;

  float leftMm_ = 0.0f;
  float rightMm_ = 0.0f;
  float rearMm_ = 0.0f;
  bool leftFresh_ = false;
  bool rightFresh_ = false;
  bool rearFresh_ = false;

  static bool bringUp(VL53L0X& sensor, int xshut, uint8_t address);
  static void sample(VL53L0X& sensor, float& held, bool& fresh, bool& valid,
                     uint16_t& rawMm);
};

// Hold every part in reset, then release them one at a time. The last release
// is the left sensor, which keeps the default address.
inline bool DistanceSensorManager::begin() {
  // All three start in reset, so no part is awake while another is addressed ---
  pinMode(board::PIN_XSHUT_L, OUTPUT);
  pinMode(board::PIN_XSHUT_R, OUTPUT);
  pinMode(board::PIN_XSHUT_REAR, OUTPUT);
  digitalWrite(board::PIN_XSHUT_L, LOW);
  digitalWrite(board::PIN_XSHUT_R, LOW);
  digitalWrite(board::PIN_XSHUT_REAR, LOW);
  delay(20);
  // end

  // Rear first, then right, then left, so the part that keeps 0x29 is last ---
  if (!bringUp(rear_, board::PIN_XSHUT_REAR, board::ADDR_REAR)) return false;
  if (!bringUp(right_, board::PIN_XSHUT_R, board::ADDR_RIGHT)) return false;
  if (!bringUp(left_, board::PIN_XSHUT_L, board::ADDR_LEFT)) return false;
  // end

  return true;
}

// Enable one part, give it its address and start continuous ranging. init()
// runs while the part is still at 0x29 and no other part is awake yet.
inline bool DistanceSensorManager::bringUp(VL53L0X& sensor, int xshut,
                                           uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(20);
  sensor.setTimeout(sensor::TIMEOUT_MS);
  if (!sensor.init()) {
    Serial.printf("sensor init failed on xshut %d, wanted 0x%02X\n", xshut,
                  address);
    return false;
  }
  sensor.setAddress(address);
  sensor.setMeasurementTimingBudget(sensor::BUDGET_US);
  sensor.startContinuous(sensor::PERIOD_MS);
  return true;
}

// Convert one reading to millimetres and blend it into the held value. The raw
// value is handed back so the boot line can show exactly what a part answered.
inline void DistanceSensorManager::sample(VL53L0X& sensor, float& held,
                                          bool& fresh, bool& valid,
                                          uint16_t& rawMm) {
  rawMm = sensor.readRangeContinuousMillimeters();
  valid = rawMm != 0 && rawMm <= sensor::MAX_VALID_MM;
  if (!valid) return;

  // Snap to the first valid value, then follow main's 0.35 EMA ---
  const float mm = (float)rawMm;
  held = fresh ? sensor::EMA_ALPHA * mm + (1.0f - sensor::EMA_ALPHA) * held : mm;
  fresh = true;
  // end
}

inline SensorReadings DistanceSensorManager::read() {
  SensorReadings r;
  sample(left_, leftMm_, leftFresh_, r.leftValid, r.rawL);
  sample(right_, rightMm_, rightFresh_, r.rightValid, r.rawR);
  sample(rear_, rearMm_, rearFresh_, r.rearValid, r.rawRear);
  r.gL = leftMm_;
  r.gR = rightMm_;
  r.gRear = rearMm_;
  return r;
}
