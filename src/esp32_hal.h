#pragma once

// The ESP32 side of the control core's Hal. Pins and sensor timing come from
// control/config.h, so the measured map lives in one place. Every VL53L0X
// powers up at 0x29, so they come up one at a time and the one keeping 0x29
// comes up last.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <esp_timer.h>

#include <cmath>

#include "config.h"
#include "hal.h"

struct Esp32Hal : Hal {
  VL53L0X left;
  VL53L0X right;
  VL53L0X rear;
  Servo servo;

  // Bring up the bus, motor, servo and sensors. False means a sensor did not answer, which the caller reports and carries on from, since the watchdogs decide.
  bool begin() {
    // The driver pins go low first, so the car cannot move while the rest of the
    // hardware comes up.
    pinMode(board::PIN_MOTOR_IN1, OUTPUT);
    pinMode(board::PIN_MOTOR_IN2, OUTPUT);
    pinMode(board::PIN_MOTOR_B1, OUTPUT);
    pinMode(board::PIN_MOTOR_B2, OUTPUT);
    digitalWrite(board::PIN_MOTOR_IN1, LOW);
    digitalWrite(board::PIN_MOTOR_IN2, LOW);
    digitalWrite(board::PIN_MOTOR_B1, LOW);
    digitalWrite(board::PIN_MOTOR_B2, LOW);

    Wire.begin(board::PIN_I2C_SDA, board::PIN_I2C_SCL);

    // 1000 to 2000 us is the datasheet table, so 90 degrees lands on the SG90's 1500 us neutral and one commanded degree is one real degree.
    servo.setPeriodHertz(50);
    servo.attach(board::PIN_SERVO, 1000, 2000);
    servo.write(90);

    // All three held in reset, then out of reset one at a time.
    pinMode(board::PIN_XSHUT_L, OUTPUT);
    pinMode(board::PIN_XSHUT_R, OUTPUT);
    pinMode(board::PIN_XSHUT_REAR, OUTPUT);
    digitalWrite(board::PIN_XSHUT_L, LOW);
    digitalWrite(board::PIN_XSHUT_R, LOW);
    digitalWrite(board::PIN_XSHUT_REAR, LOW);
    delay(20);

    bool ok = true;
    ok = bringUp(rear, board::PIN_XSHUT_REAR, board::ADDR_REAR) && ok;
    ok = bringUp(right, board::PIN_XSHUT_R, board::ADDR_RIGHT) && ok;
    ok = bringUp(left, board::PIN_XSHUT_L, board::ADDR_LEFT) && ok;
    return ok;
  }

  void readSensors(float& gL, float& gR, float& gRear) override {
    // Millimetres to metres, no clamping. 8190 for no target and 65535 for a timeout both land outside geom::validRange, which is what the core wants.
    gL = metres(left.readRangeContinuousMillimeters());
    gR = metres(right.readRangeContinuousMillimeters());
    gRear = metres(rear.readRangeContinuousMillimeters());
  }

  void setServo(float deg) override {
    // A whole degree is the smallest step the servo is driven at, so the profile is rounded here and the telemetry carries the same number.
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    servo.write((int)lroundf(deg));
  }

  void setMotor(int dir, uint8_t duty = 255) override {
    // Both channels take the same direction, each with its own inversion, so the
    // car tracks straight on paper. A curve here is motor mismatch, which belongs
    // in telemetry and not in a trim.
    driveChannel(board::PIN_MOTOR_IN1, board::PIN_MOTOR_IN2, dir, duty, board::INVERT_LEFT_MOTOR);
    driveChannel(board::PIN_MOTOR_B1, board::PIN_MOTOR_B2, dir, duty, board::INVERT_RIGHT_MOTOR);
  }

  float now() override {
    return (float)((double)esp_timer_get_time() / 1000000.0);
  }

 private:
  // One DRV8833 channel. Every write is analogWrite because the first PWM on a
  // pin hands it to the LEDC peripheral, and digitalWrite then refuses the pin.
  // dir 0 writes zero to both inputs, which floats the outputs and is the gentle
  // brake the rear limit wants.
  static void driveChannel(int inA, int inB, int dir, uint8_t duty, bool invert) {
    if (dir == 0) {
      analogWrite(inA, 0);
      analogWrite(inB, 0);
      return;
    }
    const bool fwd = (dir > 0) != invert;
    if (fwd) {
      analogWrite(inA, duty);
      analogWrite(inB, 0);
    } else {
      analogWrite(inA, 0);
      analogWrite(inB, duty);
    }
  }

  // Enable one sensor, move it off 0x29 into continuous mode. init() runs before setAddress() because the device is still at 0x29 and no other is awake.
  static bool bringUp(VL53L0X& s, int xshut, uint8_t addr) {
    digitalWrite(xshut, HIGH);
    delay(20);
    s.setTimeout(board::SENSOR_TIMEOUT_MS);
    if (!s.init()) return false;
    s.setAddress(addr);
    s.setMeasurementTimingBudget(board::SENSOR_BUDGET_US);
    s.startContinuous(board::SENSOR_PERIOD_MS);
    return true;
  }

  static float metres(uint16_t mm) { return (float)mm * 0.001f; }
};
