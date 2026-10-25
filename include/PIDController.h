#pragma once
#include <Arduino.h>
#include <math.h>
#include "ParkingConfig.h"

class PIDController {
public:
    PIDController(float p = 1.6f, float i = 0.05f, float d = 0.45f)
        : Kp(p), Ki(i), Kd(d),
          integral(0.0f), previousError(0.0f),
          filteredDerivative(0.0f),
          maxIntegral(25.0f),
          derivativeFilterAlpha(0.6f) {}

    void setTunings(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    void reset() {
        integral = 0.0f;
        previousError = 0.0f;
        filteredDerivative = 0.0f;
    }

    // Computes control output in degrees offset from center [-45, +45]
    float compute(float error, float deltaTimeSec) {
        if (deltaTimeSec <= 0.001f) deltaTimeSec = 0.01f;

        // Apply Deadband to eliminate jitter around perfect center
        if (fabs(error) < DEADBAND_CM) {
            error = 0.0f;
        }

        // 1. Proportional Term
        float pTerm = Kp * error;

        // 2. Integral Term with Anti-Windup Clamping
        if (error != 0.0f) {
            integral += error * deltaTimeSec;
            integral = constrain(integral, -maxIntegral, maxIntegral);
        } else {
            // Decay integral slowly when in deadband
            integral *= 0.95f;
        }
        float iTerm = Ki * integral;

        // 3. Derivative Term with Low-Pass Filtering
        float rawDerivative = (error - previousError) / deltaTimeSec;
        filteredDerivative = (derivativeFilterAlpha * rawDerivative) + 
                             ((1.0f - derivativeFilterAlpha) * filteredDerivative);
        float dTerm = Kd * filteredDerivative;

        previousError = error;

        // Total control output (degrees offset)
        return pTerm + iTerm + dTerm;
    }

    // Maps control output to the physical servo angle and Ackerman wheel angles
    static int mapToServoAngle(float controlOutput, bool inverted = STEERING_INVERTED) {
        if (inverted) {
            controlOutput = -controlOutput;
        }
        int targetAngle = SERVO_CENTER_DEG + (int)round(controlOutput);
        return constrain(targetAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
    }

    // Calculate real road wheel angles based on user's Ackerman specifications:
    // Max rotation (180° / 0°) gives 40° inner and 32° outer wheel rotation
    static void getAckermanWheelAngles(int servoAngle, float &leftWheelDeg, float &rightWheelDeg) {
        float deflectionRatio = (float)(servoAngle - SERVO_CENTER_DEG) / (float)(SERVO_MAX_DEG - SERVO_CENTER_DEG);
        deflectionRatio = constrain(deflectionRatio, -1.0f, 1.0f);

        if (deflectionRatio >= 0.0f) {
            // Steering Right: Right wheel is INNER, Left wheel is OUTER
            rightWheelDeg = deflectionRatio * ACKERMAN_INNER_MAX_DEG;
            leftWheelDeg  = deflectionRatio * ACKERMAN_OUTER_MAX_DEG;
        } else {
            // Steering Left: Left wheel is INNER, Right wheel is OUTER
            float absRatio = -deflectionRatio;
            leftWheelDeg  = -(absRatio * ACKERMAN_INNER_MAX_DEG);
            rightWheelDeg = -(absRatio * ACKERMAN_OUTER_MAX_DEG);
        }
    }

private:
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previousError;
    float filteredDerivative;
    float maxIntegral;
    float derivativeFilterAlpha;
};
