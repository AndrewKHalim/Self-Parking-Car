// Three VL53L0X on one I2C bus. They all power up on 0x29, so they are held in
// reset by XSHUT and released one at a time, each re-addressed before the next
// one wakes up.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "config.h"

struct SensorReading {
    uint16_t gL_mm;
    uint16_t gR_mm;
    uint16_t rear_mm;
    bool left_valid;
    bool right_valid;
    bool rear_valid;
};

class SensorBank {
public:
    bool begin() {
        pinMode(pins::XSHUT_LEFT, OUTPUT);
        pinMode(pins::XSHUT_RIGHT, OUTPUT);
        pinMode(pins::XSHUT_REAR, OUTPUT);
        digitalWrite(pins::XSHUT_LEFT, LOW);
        digitalWrite(pins::XSHUT_RIGHT, LOW);
        digitalWrite(pins::XSHUT_REAR, LOW);
        delay(20);  // datasheet reset pulse

        Wire.begin(pins::I2C_SDA, pins::I2C_SCL);

        // Order matters only in that each sensor must be addressed while it is
        // the sole device answering on 0x29.
        if (!bringUp(rear_, pins::XSHUT_REAR, sensors::ADDR_REAR)) {
            return false;
        }
        if (!bringUp(right_, pins::XSHUT_RIGHT, sensors::ADDR_RIGHT)) {
            return false;
        }
        return bringUp(left_, pins::XSHUT_LEFT, sensors::ADDR_LEFT);
    }

    // Blocking by design: the control loop runs at 30 Hz and a 33 ms timing
    // budget plus a 50 ms timeout fits inside one period.
    SensorReading read() {
        SensorReading r = {};
        r.rear_mm = rear_.readRangeContinuousMillimeters();
        r.gR_mm = right_.readRangeContinuousMillimeters();
        r.gL_mm = left_.readRangeContinuousMillimeters();

        // A side sensor looking into open space is not a usable distance.
        r.left_valid = r.gL_mm > 0 && r.gL_mm < sensors::OUT_OF_RANGE_MM;
        r.right_valid = r.gR_mm > 0 && r.gR_mm < sensors::OUT_OF_RANGE_MM;
        // The rear sensor reads 0 when the wall is closer than its minimum
        // range, which is a valid and very much "stop now" answer.
        r.rear_valid = r.rear_mm < sensors::OUT_OF_RANGE_MM;
        return r;
    }

private:
    bool bringUp(VL53L0X& sensor, int xshut_pin, uint8_t address) {
        digitalWrite(xshut_pin, HIGH);
        delay(10);  // boot time after reset release
        sensor.setTimeout(sensors::TIMEOUT_MS);
        if (!sensor.init()) {
            return false;
        }
        sensor.setAddress(address);
        sensor.setMeasurementTimingBudget(sensors::TIMING_BUDGET_US);
        sensor.startContinuous(sensors::PERIOD_MS);
        return true;
    }

    VL53L0X left_;
    VL53L0X right_;
    VL53L0X rear_;
};
