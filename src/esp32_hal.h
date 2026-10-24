#pragma once

// the esp32 side of the control core's Hal. one translation unit includes this,
// so the three sensors, the servo and the motor pins live in the struct rather
// than as globals.
//
// the sensors share one i2c bus and differ only in address. every vl53l0x
// powers up at 0x29, so they are brought up one at a time, and the sensor that
// keeps 0x29 is brought up last: an address write for any other sensor would
// otherwise land on it too, because both would be listening at 0x29.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <esp_timer.h>

#include <cmath>

#include "hal.h"

// N16R8 pin map. gpio 26 to 32 belong to the octal psram, 19 and 20 to usb,
// and 0, 3, 45 and 46 are strapping pins sampled at boot, so none of them
// appear here. ---
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr int PIN_XSHUT_L = 15;
constexpr int PIN_XSHUT_R = 16;
constexpr int PIN_XSHUT_REAR = 10;
constexpr int PIN_SERVO = 17;
constexpr int PIN_MOTOR_IN1 = 18;
constexpr int PIN_MOTOR_IN2 = 21;
// end

// the address each sensor ends up on. 0x29 is what they all start at. ---
constexpr uint8_t ADDR_LEFT = 0x29;
constexpr uint8_t ADDR_RIGHT = 0x2A;
constexpr uint8_t ADDR_REAR = 0x2B;
// end

// the control loop ticks at 30 hz, so a measurement is asked for every 33 ms
// and the timing budget matches it. a much longer budget would hand the loop
// the same sample twice, and a period shorter than the budget would race the
// sensor's own sequence. ---
constexpr uint32_t SENSOR_BUDGET_US = 33000;
constexpr uint32_t SENSOR_PERIOD_MS = 33;
// end

// the driver defaults to no timeout, which blocks the control task forever on
// a sensor that stopped answering. 50 ms is more than one measurement takes, so
// a healthy sensor never trips it, and a dead one costs a bounded delay instead
// of a stalled loop. ---
constexpr uint16_t SENSOR_TIMEOUT_MS = 50;
// end

struct Esp32Hal : Hal {
  VL53L0X left;
  VL53L0X right;
  VL53L0X rear;
  Servo servo;

  // bring up the bus, the motor pins, the servo and the three sensors. false
  // means at least one sensor did not answer; the caller reports it and keeps
  // going, because the supervisor's own watchdogs are what decide whether the
  // car may move without a reading. ---
  bool begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // the driver pins are driven low first, so the car cannot move while the
    // rest of the hardware comes up.
    pinMode(PIN_MOTOR_IN1, OUTPUT);
    pinMode(PIN_MOTOR_IN2, OUTPUT);
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    // end

    // 1000 and 2000 us, which is the datasheet's 1 to 2 ms position table.
    // 90 degrees is the midpoint of the table, so it lands on the 1500 us the
    // sg90 calls neutral, and because the table's 180 degrees span exactly
    // that window one commanded degree is one real degree. a wider pulse range
    // would put 90 degrees off neutral and compress every angle the loop asks
    // for. ---
    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, 1000, 2000);
    servo.write(90);
    // end

    // all three held in reset, then out of reset one at a time. ---
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
    // end
  }

  void readSensors(float& gL, float& gR, float& gRear) override {
    // millimetres to metres with no clamping. a sensor that sees nothing
    // reports 8190, which is 8.190 m, and a read that times out reports 65535.
    // both sit outside geom::validRange, which is what the control core needs:
    // a reading it cannot trust has to arrive as one, not as a plausible
    // number it would steer on. ---
    gL = metres(left.readRangeContinuousMillimeters());
    gR = metres(right.readRangeContinuousMillimeters());
    gRear = metres(rear.readRangeContinuousMillimeters());
    // end
  }

  void setServo(float deg) override {
    // a degree is the smallest step the servo is driven at, so the profile is
    // rounded here and the telemetry carries the same whole number. ---
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    servo.write((int)lroundf(deg));
    // end
  }

  void setMotor(int dir) override {
    // the drv8833 truth table. one input high drives and which one sets the
    // direction. the hal calls 0 a brake and both low is the gentle way to
    // obey it: the outputs float, so the motor coasts and the gearbox is not
    // asked to absorb the stop. both high is the hard brake, and it stays in
    // reserve until the stopping distance is measured rather than assumed. ---
    if (dir > 0) {
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
    } else if (dir < 0) {
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
    } else {
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, LOW);
    }
    // end
  }

  float now() override {
    return (float)((double)esp_timer_get_time() / 1000000.0);
  }

 private:
  // enable one sensor out of reset, initialise it, move it off 0x29 and put it
  // into timed continuous mode. init() runs before setAddress() because the
  // device is still at 0x29 at that point, and no other sensor is awake. ---
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
  // end

  static float metres(uint16_t mm) { return (float)mm * 0.001f; }
};
