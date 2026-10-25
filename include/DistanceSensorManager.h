#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "ParkingConfig.h"

struct SensorReadings {
    float rawLeftCm;
    float rawRightCm;
    float rawBackCm;

    float filteredLeftCm;
    float filteredRightCm;
    float filteredBackCm;

    float lateralLeftCm;
    float lateralRightCm;

    bool leftValid;
    bool rightValid;
    bool backValid;
    bool proximityAlert; // Triggered if any obstacle is dangerously close (< 3.5 cm)
};

class DistanceSensorManager {
public:
    DistanceSensorManager() 
        : filteredLeft(SENSOR_MAX_VALID_CM),
          filteredRight(SENSOR_MAX_VALID_CM),
          filteredBack(SENSOR_MAX_VALID_CM),
          leftInitialized(false),
          rightInitialized(false),
          backInitialized(false) {}

    bool begin() {
        pinMode(PIN_XSHUT_LEFT, OUTPUT);
        pinMode(PIN_XSHUT_RIGHT, OUTPUT);
        pinMode(PIN_XSHUT_BACK, OUTPUT);

        // Put all 3 sensors into reset mode
        digitalWrite(PIN_XSHUT_LEFT, LOW);
        digitalWrite(PIN_XSHUT_RIGHT, LOW);
        digitalWrite(PIN_XSHUT_BACK, LOW);
        delay(15);

        // 1. Initialize Left Sensor (Address 0x30)
        digitalWrite(PIN_XSHUT_LEFT, HIGH);
        delay(15);
        if (!loxLeft.begin(ADDR_VL53L0X_LEFT, false, &Wire)) {
            Serial.println("❌ ERROR: Left VL53L0X (0x30) failed to initialize!");
            return false;
        }
        leftInitialized = true;
        Serial.println("✓ Left VL53L0X (0x30) initialized.");

        // 2. Initialize Right Sensor (Address 0x31)
        digitalWrite(PIN_XSHUT_RIGHT, HIGH);
        delay(15);
        if (!loxRight.begin(ADDR_VL53L0X_RIGHT, false, &Wire)) {
            Serial.println("❌ ERROR: Right VL53L0X (0x31) failed to initialize!");
            return false;
        }
        rightInitialized = true;
        Serial.println("✓ Right VL53L0X (0x31) initialized.");

        // 3. Initialize Back Sensor (Address 0x32)
        digitalWrite(PIN_XSHUT_BACK, HIGH);
        delay(15);
        if (!loxBack.begin(ADDR_VL53L0X_BACK, false, &Wire)) {
            Serial.println("❌ ERROR: Back Center VL53L0X (0x32) failed to initialize!");
            return false;
        }
        backInitialized = true;
        Serial.println("✓ Back Center VL53L0X (0x32) initialized.");

        return true;
    }

    SensorReadings read() {
        SensorReadings readings;
        readings.proximityAlert = false;

        // Process Left Sensor
        readings.rawLeftCm = processSingleSensor(loxLeft, filteredLeft, readings.leftValid, readings.proximityAlert);
        readings.filteredLeftCm = filteredLeft;
        readings.lateralLeftCm = readings.filteredLeftCm * SIN_45_DEG;

        // Process Right Sensor
        readings.rawRightCm = processSingleSensor(loxRight, filteredRight, readings.rightValid, readings.proximityAlert);
        readings.filteredRightCm = filteredRight;
        readings.lateralRightCm = readings.filteredRightCm * SIN_45_DEG;

        // Process Back Center Sensor (straight rear, no trig scale needed)
        readings.rawBackCm = processSingleSensor(loxBack, filteredBack, readings.backValid, readings.proximityAlert);
        readings.filteredBackCm = filteredBack;

        return readings;
    }

private:
    Adafruit_VL53L0X loxLeft;
    Adafruit_VL53L0X loxRight;
    Adafruit_VL53L0X loxBack;

    float filteredLeft;
    float filteredRight;
    float filteredBack;

    bool leftInitialized;
    bool rightInitialized;
    bool backInitialized;

    float processSingleSensor(Adafruit_VL53L0X &sensor, float &filteredVal, bool &isValid, bool &globalProximityAlert) {
        VL53L0X_RangingMeasurementData_t measure;
        sensor.rangingTest(&measure, false);

        float distCm = SENSOR_MAX_VALID_CM;
        isValid = false;

        // In Adafruit VL53L0X:
        // Status 4: Out of Range / Phase Failure
        // Status 3: Min Range Fail (< 30 mm)
        // Status 0 (Valid), 1 (Sigma warning), 2 (Signal warning) contain valid millimeter readings
        if (measure.RangeStatus != 4) {
            distCm = measure.RangeMilliMeter / 10.0f;

            if (distCm < SENSOR_MIN_EFFECTIVE_CM || measure.RangeStatus == 3) {
                // Obstacle too close (< 3.0 cm)
                distCm = SENSOR_MIN_EFFECTIVE_CM;
                globalProximityAlert = true;
                isValid = true;
            } else if (distCm <= SENSOR_MAX_VALID_CM) {
                if (distCm < SENSOR_EMERGENCY_CM) {
                    globalProximityAlert = true;
                }
                isValid = true;
            }
        } else {
            // Status 4: Out of range / phase fail
            // If car was already very close (< 6 cm), phase fail is often optical saturation from being under 30 mm
            if (filteredVal < 6.0f) {
                distCm = SENSOR_MIN_EFFECTIVE_CM;
                globalProximityAlert = true;
                isValid = true;
            } else {
                distCm = SENSOR_MAX_VALID_CM;
                isValid = false;
            }
        }

        // Apply Exponential Moving Average (EMA) Filter
        if (isValid) {
            // Fast initialization: snap directly to distance on first valid reading
            if (filteredVal >= (SENSOR_MAX_VALID_CM - 1.0f)) {
                filteredVal = distCm;
            } else {
                filteredVal = (SENSOR_EMA_ALPHA * distCm) + ((1.0f - SENSOR_EMA_ALPHA) * filteredVal);
            }
        }

        return distCm;
    }
};
