#pragma once
#include <Arduino.h>

// ============================================================================
// Hardware Pin Definitions (ESP32-S3)
// ============================================================================
constexpr uint8_t PIN_I2C_SDA          = 8;
constexpr uint8_t PIN_I2C_SCL          = 9;

// VL53L0X XSHUT Pins (Sequential power-on assignment)
constexpr uint8_t PIN_XSHUT_LEFT       = 6;   // Left 45° ToF Sensor
constexpr uint8_t PIN_XSHUT_BACK       = 5;   // Back Center 0° ToF Sensor
constexpr uint8_t PIN_XSHUT_RIGHT      = 4;   // Right 45° ToF Sensor

// Actuator Pins
constexpr uint8_t PIN_SERVO_STEERING   = 7;   // Steering Servo

// Dual Rear Drive Motors (L298N with ENA/ENB jumpers installed)
constexpr bool    MOTOR_ENABLED        = true;
// Left Motor Pins (L298N IN1, IN2)
constexpr uint8_t PIN_MOTOR_L_IN1      = 16;
constexpr uint8_t PIN_MOTOR_L_IN2      = 17;
// Right Motor Pins (L298N IN3, IN4)
constexpr uint8_t PIN_MOTOR_R_IN1      = 14;
constexpr uint8_t PIN_MOTOR_R_IN2      = 13;

// Motor Polarity (toggle to true if a motor spins backwards)
constexpr bool    INVERT_LEFT_MOTOR    = true;  // Inverted so reverse drives backward
constexpr bool    INVERT_RIGHT_MOTOR   = false;

// ============================================================================
// I2C Sensor Addresses
// ============================================================================
// Default factory address is 0x29; reassigned via XSHUT during initialization
constexpr uint8_t ADDR_VL53L0X_LEFT    = 0x30;
constexpr uint8_t ADDR_VL53L0X_RIGHT   = 0x31;
constexpr uint8_t ADDR_VL53L0X_BACK    = 0x32;

// ============================================================================
// Steering Geometry & Ackerman Setup
// ============================================================================
constexpr int   SERVO_CENTER_DEG       = 90;  // Straight ahead
constexpr int   SERVO_MIN_DEG          = 45;  // Maximum left servo limit (can be 0 for full travel)
constexpr int   SERVO_MAX_DEG          = 135; // Maximum right servo limit (can be 180 for full travel)
constexpr bool  STEERING_INVERTED      = false; // Set to true if servo horn is inverted

// User's custom Ackerman linkage specifications:
// At maximum rotation (180° / 0°), inner wheel reaches 40° and outer wheel reaches 32°
constexpr float ACKERMAN_INNER_MAX_DEG = 40.0f;
constexpr float ACKERMAN_OUTER_MAX_DEG = 32.0f;

// ============================================================================
// Sensor Geometry & Trigonometry
// ============================================================================
// Side sensors mounted at 45° to the car longitudinal midline
constexpr float SENSOR_ANGLE_DEG       = 45.0f;
constexpr float SIN_45_DEG             = 0.70710678f; // sin(45°) = cos(45°)
constexpr float COS_45_DEG             = 0.70710678f;

// Sensor Distance Limits (in cm)
constexpr float SENSOR_MIN_EFFECTIVE_CM= 3.0f;   // 30 mm physical VL53L0X limit
constexpr float SENSOR_EMERGENCY_CM    = 4.0f;   // Proximity emergency stop threshold
constexpr float SENSOR_MAX_VALID_CM    = 150.0f; // Max valid range for lateral walls
constexpr float SENSOR_EMA_ALPHA       = 0.35f;  // Exponential filter smoothing factor (0..1)

// ============================================================================
// Parking Target & State Machine Thresholds (in cm)
// ============================================================================
constexpr float TARGET_BACK_WALL_CM    = 7.0f;   // Stop distance from rear wall
constexpr float BACK_WALL_TOLERANCE_CM = 0.6f;   // Allowable stop window: [6.4, 7.6] cm
constexpr float APPROACH_SLOW_DIST_CM  = 16.0f;  // Switch to crawl speed near rear wall
constexpr float PULL_FORWARD_THRESH_CM = 13.0f;  // Trigger forward correction if misaligned
constexpr float ALIGNMENT_TOLERANCE_CM = 1.2f;   // Target lateral centering accuracy (|L - R|)
constexpr float DEADBAND_CM            = 0.8f;   // Error deadband to prevent servo jitter

// Vehicle Dimensions (in cm)
constexpr float CAR_WIDTH_CM           = 18.5f; // 185 mm
constexpr float CAR_LENGTH_CM          = 23.4f; // 234 mm

// Speeds (PWM Duty 0-255)
constexpr uint8_t SPEED_REVERSE_CRUISE = 110;    // Standard reversing speed
constexpr uint8_t SPEED_REVERSE_CRAWL  = 70;     // Precision docking crawl
constexpr uint8_t SPEED_FORWARD_ADJUST = 90;     // Pull-forward adjustment speed
constexpr unsigned long PULL_FWD_DURATION_MS = 1400; // Duration to pull forward
constexpr float PULL_FORWARD_EXIT_DIST_CM = 26.0f;   // Distance from back wall to complete pull forward

