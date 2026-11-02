// DRV8833 dual H-bridge and the steering servo. Both motor inputs are driven
// with PWM on one leg and 0 on the other (fast decay); brake shorts both legs.
#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

#include "config.h"

class Actuators {
public:
    void begin() {
        ledcAttach(pins::MOTOR_L_IN1, drive::PWM_HZ, drive::PWM_BITS);
        ledcAttach(pins::MOTOR_L_IN2, drive::PWM_HZ, drive::PWM_BITS);
        ledcAttach(pins::MOTOR_R_IN1, drive::PWM_HZ, drive::PWM_BITS);
        ledcAttach(pins::MOTOR_R_IN2, drive::PWM_HZ, drive::PWM_BITS);
        brake();  // never start powered

        servo_.attach(pins::SERVO);
        servo_.write(steering::SERVO_CENTRE_DEG);
        current_servo_deg_ = steering::SERVO_CENTRE_DEG;
    }

    // Both legs high: the DRV8833 shorts each winding. This is why the car
    // stops at the rear target instead of coasting past it.
    void brake() {
        ledcWrite(pins::MOTOR_L_IN1, 255);
        ledcWrite(pins::MOTOR_L_IN2, 255);
        ledcWrite(pins::MOTOR_R_IN1, 255);
        ledcWrite(pins::MOTOR_R_IN2, 255);
    }

    void forward(uint8_t duty) {
        driveMotor(pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, pins::INVERT_LEFT_MOTOR, duty);
        driveMotor(pins::MOTOR_R_IN1, pins::MOTOR_R_IN2, pins::INVERT_RIGHT_MOTOR, duty);
    }

    void reverse(uint8_t duty) {
        driveMotor(pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, !pins::INVERT_LEFT_MOTOR, duty);
        driveMotor(pins::MOTOR_R_IN1, pins::MOTOR_R_IN2, !pins::INVERT_RIGHT_MOTOR, duty);
    }

    int setServoDeg(int deg) {
        const int clamped = util::clampi(deg, steering::SERVO_MIN_DEG, steering::SERVO_MAX_DEG);
        servo_.write(clamped);
        current_servo_deg_ = clamped;
        return clamped;
    }

    // Road wheel degrees in, servo degrees out: the pinion/rack/horn linkage
    // multiplies by 2.92, so 15.4 degrees of wheel is the whole servo travel.
    int setWheelAngleDeg(float wheel_deg) {
        const float clamped =
            util::clampf(wheel_deg, -steering::MAX_WHEEL_DEG, steering::MAX_WHEEL_DEG);
        const int servo_deg =
            steering::SERVO_CENTRE_DEG +
            (int)lroundf((float)steering::STEER_SIGN * clamped * steering::SERVO_PER_WHEEL_DEG);
        return setServoDeg(servo_deg);
    }

    int servoDeg() const { return current_servo_deg_; }

private:
    void driveMotor(int in1, int in2, bool forward_dir, uint8_t duty) {
        ledcWrite(in1, forward_dir ? duty : 0);
        ledcWrite(in2, forward_dir ? 0 : duty);
    }

    Servo servo_;
    int current_servo_deg_ = steering::SERVO_CENTRE_DEG;
};
