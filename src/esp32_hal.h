#pragma once

// The ESP32 side of the control core's Hal, so pins live in the struct rather than as globals. Every VL53L0X powers up at 0x29, so they come up one at a time and the one keeping 0x29 comes up last.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <esp_timer.h>

#include <cmath>

#include "hal.h"

// N16R8 pin map. GPIO 26 to 32 are octal PSRAM, 19 and 20 USB, and 0, 3, 45, 46 are strapping pins sampled at boot, so none appear.
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr int PIN_XSHUT_L = 15;
constexpr int PIN_XSHUT_R = 16;
constexpr int PIN_XSHUT_REAR = 10;
constexpr int PIN_SERVO = 17;
constexpr int PIN_MOTOR_IN1 = 18;
constexpr int PIN_MOTOR_IN2 = 21;
constexpr int PIN_MOTOR_B1 = 4;
constexpr int PIN_MOTOR_B2 = 5;

// The two motors are mirrored, so one channel has to drive flipped or the car
// spins on the spot. Channel A on 18 and 21 is the left motor, B on 4 and 5 the
// right. Both flags came from the main branch's pair and are settled by a
// forward pulse with the wheels off the ground.
constexpr bool INVERT_LEFT_MOTOR = true;
constexpr bool INVERT_RIGHT_MOTOR = false;

// The address each sensor ends up on. 0x29 is where they all start.
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;

// 30 Hz, so a measurement every 33 ms, with the budget to match. Longer hands the loop the same sample twice, shorter races the sensor's own sequence.
constexpr uint32_t SENSOR_BUDGET_US = 33000;
constexpr uint32_t SENSOR_PERIOD_MS = 33;

// The driver's default timeout is none, which blocks on a dead sensor. 50 ms outlasts one measurement, so a healthy sensor never trips it.
constexpr uint16_t SENSOR_TIMEOUT_MS = 50;

struct Esp32Hal : Hal {
  VL53L0X left;
  VL53L0X right;
  VL53L0X rear;
  Servo servo;

  // Bring up the bus, motor, servo and sensors. False means a sensor did not answer, which the caller reports and carries on from, since the watchdogs decide.
  bool begin() {
    // The driver pins go low first, so the car cannot move while the rest of the
    // hardware comes up.
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    pinMode(PIN_MOTOR_B1, OUTPUT);
    pinMode(PIN_MOTOR_B2, OUTPUT);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    digitalWrite(PIN_MOTOR_B1, LOW);
    digitalWrite(PIN_MOTOR_B2, LOW);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // 1000 to 2000 us is the datasheet table, so 90 degrees lands on the SG90's 1500 us neutral and one commanded degree is one real degree.
    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, 1000, 2000);
    servo.write(90);

    // All three held in reset, then out of reset one at a time.
    pinMode(PIN_XSHUT_L, OUTPUT);
    pinMode(PIN_XSHUT_R, OUTPUT);
    pinMode(PIN_XSHUT_REAR, OUTPUT);
    digitalWrite(PIN_XSHUT_L, LOW);
    digitalWrite(PIN_XSHUT_R, LOW);
    digitalWrite(PIN_XSHUT_REAR, LOW);
    delay(20);

    bool ok = true;
    ok = bringUp(rear, PIN_XSHUT_REAR, ADDR_REAR) && ok;
    ok = bringUp(right, PIN_XSHUT_R, ADDR_RIGHT) && ok;
    ok = bringUp(left, PIN_XSHUT_L, ADDR_LEFT) && ok;
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
    driveChannel(PIN_MOTOR_IN1, PIN_MOTOR_IN2, dir, duty, INVERT_LEFT_MOTOR);
    driveChannel(PIN_MOTOR_B1, PIN_MOTOR_B2, dir, duty, INVERT_RIGHT_MOTOR);
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
    s.setTimeout(SENSOR_TIMEOUT_MS);
    if (!s.init()) return false;
    s.setAddress(addr);
    s.setMeasurementTimingBudget(SENSOR_BUDGET_US);
    s.startContinuous(SENSOR_PERIOD_MS);
    return true;
  }

  static float metres(uint16_t mm) { return (float)mm * 0.001f; }
};
