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

        pinMode(PIN_MOTOR_L_IN1, OUTPUT);
        pinMode(PIN_MOTOR_L_IN2, OUTPUT);
        pinMode(PIN_MOTOR_R_IN1, OUTPUT);
        pinMode(PIN_MOTOR_R_IN2, OUTPUT);

        stop();
        Serial.printf("✓ Dual L298N motors initialized: Left(IN1=%d, IN2=%d), Right(IN3=%d, IN4=%d)\n",
                      PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2);
    }

    void reverse(uint8_t speed) {
        currentSpeed = speed;
        currentDir = MOTOR_REVERSE;
        if (!MOTOR_ENABLED) return;

        // Drive Left Motor Reverse
        setMotorOutput(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, speed, true, INVERT_LEFT_MOTOR);
        // Drive Right Motor Reverse
        setMotorOutput(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, speed, true, INVERT_RIGHT_MOTOR);
    }

    void forward(uint8_t speed) {
        currentSpeed = speed;
        currentDir = MOTOR_FORWARD;
        if (!MOTOR_ENABLED) return;

        // Drive Left Motor Forward
        setMotorOutput(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, speed, false, INVERT_LEFT_MOTOR);
        // Drive Right Motor Forward
        setMotorOutput(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, speed, false, INVERT_RIGHT_MOTOR);
    }

    void stop() {
        currentSpeed = 0;
        currentDir = MOTOR_STOPPED;
        if (!MOTOR_ENABLED) return;

        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
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

    // Direct PWM on IN pins (for L298N with ENA/ENB jumpers to 5V)
    void setMotorOutput(uint8_t pin1, uint8_t pin2, uint8_t speed, bool isReverse, bool invert) {
        bool forwardDirection = !isReverse;
        if (invert) {
            forwardDirection = !forwardDirection;
        }

        if (forwardDirection) {
            analogWrite(pin1, speed);
            digitalWrite(pin2, LOW);
        } else {
            digitalWrite(pin1, LOW);
            analogWrite(pin2, speed);
        }
    }
};
