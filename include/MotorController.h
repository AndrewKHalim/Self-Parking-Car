
#pragma once
#include <Arduino.h>
#include "ParkingConfig.h"

enum MotorDirection {
    MOTOR_STOPPED = 0,
    MOTOR_REVERSE,
    MOTOR_FORWARD
};

class MotorController {
public:
    MotorController() : currentSpeed(0), currentDir(MOTOR_STOPPED) {}

    void begin() {
        if (!MOTOR_ENABLED) {
            Serial.println("ℹ Motor controller disabled in config (telemetry-only mode).");
            return;
        }

        pinMode(PIN_MOTOR_PWM, OUTPUT);
        pinMode(PIN_MOTOR_DIR1, OUTPUT);
        pinMode(PIN_MOTOR_DIR2, OUTPUT);

        stop();
        Serial.println("✓ Motor controller initialized on pins PWM=" + String(PIN_MOTOR_PWM) + 
                       ", DIR1=" + String(PIN_MOTOR_DIR1) + ", DIR2=" + String(PIN_MOTOR_DIR2));
    }

    void reverse(uint8_t speed) {
        currentSpeed = speed;
        currentDir = MOTOR_REVERSE;
        if (!MOTOR_ENABLED) return;

        digitalWrite(PIN_MOTOR_DIR1, LOW);
        digitalWrite(PIN_MOTOR_DIR2, HIGH);
        analogWrite(PIN_MOTOR_PWM, speed);
    }

    void forward(uint8_t speed) {
        currentSpeed = speed;
        currentDir = MOTOR_FORWARD;
        if (!MOTOR_ENABLED) return;

        digitalWrite(PIN_MOTOR_DIR1, HIGH);
        digitalWrite(PIN_MOTOR_DIR2, LOW);
        analogWrite(PIN_MOTOR_PWM, speed);
    }

    void stop() {
        currentSpeed = 0;
        currentDir = MOTOR_STOPPED;
        if (!MOTOR_ENABLED) return;

        digitalWrite(PIN_MOTOR_DIR1, LOW);
        digitalWrite(PIN_MOTOR_DIR2, LOW);
        analogWrite(PIN_MOTOR_PWM, 0);
    }

    MotorDirection getDirection() const { return currentDir; }
    uint8_t getSpeed() const { return currentSpeed; }

    const char* getDirectionString() const {
        switch (currentDir) {
            case MOTOR_REVERSE: return "REV";
            case MOTOR_FORWARD: return "FWD";
            default: return "STOP";
        }
    }

private:
    uint8_t currentSpeed;
    MotorDirection currentDir;
};
