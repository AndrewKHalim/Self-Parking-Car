#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "ParkingConfig.h"

// The three VL53L0X sensors on one bus. Every part powers up at 0x29, so they
// are held in reset together and released one at a time. The rear sensor is
// released first, then the right, and the left last because the left is the one
// that keeps 0x29. Ranging runs continuously, so the loop only reads the latest
// result. A reading of 8190 mm or 65535 mm means no target or a timeout, so it
// is rejected and the last good value is held with its validity cleared.

struct SensorReadings {
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

  float leftM_ = 0.0f;
  float rightM_ = 0.0f;
  float rearM_ = 0.0f;

  static bool bringUp(VL53L0X& sensor, int xshut, uint8_t address);
  static float metres(VL53L0X& sensor, float& held, bool& valid);
};

// Hold every part in reset, then release them one at a time and move each off
// 0x29 except the last. The last release is the left sensor, which keeps the
// default address.
inline bool DistanceSensorManager::begin() {
  // Everything starts in reset, so no part is awake while another is being
  // addressed.
  pinMode(board::PIN_XSHUT_L, OUTPUT);
  pinMode(board::PIN_XSHUT_R, OUTPUT);
  pinMode(board::PIN_XSHUT_REAR, OUTPUT);
  digitalWrite(board::PIN_XSHUT_L, LOW);
  digitalWrite(board::PIN_XSHUT_R, LOW);
  digitalWrite(board::PIN_XSHUT_REAR, LOW);
  delay(20);
  // end

  // Release rear, then right, then left, so the part that keeps 0x29 is awake
  // last.
  if (!bringUp(rear_, board::PIN_XSHUT_REAR, board::ADDR_REAR)) return false;
  if (!bringUp(right_, board::PIN_XSHUT_R, board::ADDR_RIGHT)) return false;
  if (!bringUp(left_, board::PIN_XSHUT_L, board::ADDR_LEFT)) return false;
  // end

  return true;
}

// Enable one part, give it its address and start continuous ranging. init()
// runs before setAddress() because the part is still at 0x29 and no other part
// is awake yet.
inline bool DistanceSensorManager::bringUp(VL53L0X& sensor, int xshut,
                                           uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(20);
  sensor.setTimeout(sensor::TIMEOUT_MS);
  if (!sensor.init()) return false;
  sensor.setAddress(address);
  sensor.setMeasurementTimingBudget(sensor::BUDGET_US);
  sensor.startContinuous(sensor::PERIOD_MS);
  return true;
}

// Convert one reading to metres and keep the last good value when the part
// reports no target or times out.
inline float DistanceSensorManager::metres(VL53L0X& sensor, float& held,
                                           bool& valid) {
  const uint16_t mm = sensor.readRangeContinuousMillimeters();
  valid = mm > 0 && mm != sensor::NO_TARGET_MM && mm != sensor::TIMEOUT_MM;
  if (valid) held = (float)mm * 0.001f;
  return held;
}

inline SensorReadings DistanceSensorManager::read() {
  SensorReadings readings;
  readings.gL = metres(left_, leftM_, readings.leftValid);
  readings.gR = metres(right_, rightM_, readings.rightValid);
  readings.gRear = metres(rear_, rearM_, readings.rearValid);
  return readings;
}
