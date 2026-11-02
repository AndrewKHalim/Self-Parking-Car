#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "config.h"

// The three VL53L0X sensors on one bus, brought up independently. All three
// power up at 0x29, so they are held in reset together and released one at a
// time. The order is rear, right, left, because the left sensor is the one that
// keeps 0x29. If a part fails its init its XSHUT line is driven low again
// before the next part is released, so a dead or miswired sensor cannot answer
// for the next one.
//
// A raw reading is fresh when it is neither the no-target code 8190 nor the
// timeout code 65535. A fresh 0 to 29 mm is the part's minimum range and means
// the face is at the wall. It is not a distance, so callers test it before they
// trust the number.

struct Readings {
  uint16_t gL;
  uint16_t gR;
  uint16_t rear;
};

class Sensors {
public:
  void begin();
  void probe();
  Readings read();

  bool initOk(int index) const { return ok_[index]; }
  uint16_t bootRaw(int index) const { return boot_[index]; }

  // The bring-up order and the identity each boot line prints.
  static constexpr int COUNT = 3;
  static const char* name(int index) {
    return index == 0 ? "rear" : (index == 1 ? "right" : "left");
  }
  static int xshutPin(int index) {
    return index == 0 ? board::PIN_XSHUT_REAR
                      : (index == 1 ? board::PIN_XSHUT_R
                                    : board::PIN_XSHUT_L);
  }
  static uint8_t address(int index) {
    return index == 0 ? board::ADDR_REAR
                      : (index == 1 ? board::ADDR_RIGHT : board::ADDR_LEFT);
  }

  static bool fresh(uint16_t mm) {
    return mm != sensor::NO_TARGET_MM && mm != sensor::TIMEOUT_MM;
  }
  static bool atWall(uint16_t mm) {
    return mm <= sensor::WALL_MAX_MM;
  }

private:
  // The index order is the bring-up order, so the boot lines read in the same
  // order the parts came up.
  VL53L0X part_[3];
  bool ok_[3] = {false, false, false};
  uint16_t boot_[3] = {0, 0, 0};

  static bool bringUp(VL53L0X& sensor, int xshut, uint8_t address);
  uint16_t readOne(int index);
};

// Hold every part in reset, then release rear, right and left in that order.
// A part that fails is put back into reset and reported, and the remaining
// parts still get their turn.
inline void Sensors::begin() {
  pinMode(board::PIN_XSHUT_L, OUTPUT);
  pinMode(board::PIN_XSHUT_R, OUTPUT);
  pinMode(board::PIN_XSHUT_REAR, OUTPUT);
  digitalWrite(board::PIN_XSHUT_L, LOW);
  digitalWrite(board::PIN_XSHUT_R, LOW);
  digitalWrite(board::PIN_XSHUT_REAR, LOW);
  delay(20);

  ok_[0] = bringUp(part_[0], board::PIN_XSHUT_REAR, board::ADDR_REAR);
  ok_[1] = bringUp(part_[1], board::PIN_XSHUT_R, board::ADDR_RIGHT);
  ok_[2] = bringUp(part_[2], board::PIN_XSHUT_L, board::ADDR_LEFT);
}

// Enable one part, give it its address and start continuous ranging. The part
// is still at 0x29 here and no other part is awake, because every earlier part
// either moved or was put back into reset.
inline bool Sensors::bringUp(VL53L0X& sensor, int xshut, uint8_t address) {
  digitalWrite(xshut, HIGH);
  delay(20);
  sensor.setTimeout(sensor::TIMEOUT_MS);
  if (!sensor.init()) {
    digitalWrite(xshut, LOW);
    return false;
  }
  sensor.setAddress(address);
  sensor.setMeasurementTimingBudget(sensor::BUDGET_US);
  sensor.startContinuous(sensor::PERIOD_MS);
  return true;
}

// One raw reading per part for the boot self-test. A part that never came up
// reads as a timeout, which is what the verdict line names.
inline void Sensors::probe() {
  boot_[0] = readOne(0);
  boot_[1] = readOne(1);
  boot_[2] = readOne(2);
}

inline uint16_t Sensors::readOne(int index) {
  if (!ok_[index]) return sensor::TIMEOUT_MM;
  return part_[index].readRangeContinuousMillimeters();
}

inline Readings Sensors::read() {
  Readings r;
  r.rear = readOne(0);
  r.gR = readOne(1);
  r.gL = readOne(2);
  return r;
}
