#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <math.h>

// Pins
#define SDA_PIN           8
#define SCL_PIN           9
#define XSHUT_LEFT_PIN    4      // Shutdown pin for Left Sensor
#define XSHUT_RIGHT_PIN   5      // Shutdown pin for Right Sensor
#define XSHUT_BACK_PIN    6      // Shutdown pin for Back Sensor
#define SERVO_PIN         13 

// I2C Addresses (Default is 0x29, must reassign during setup)
#define L0X_LEFT_ADDR     0x30
#define L0X_RIGHT_ADDR    0x31
#define L0X_BACK_ADDR     0x32

// Sensor Objects & Servo
Adafruit_VL53L0X lox_left;
Adafruit_VL53L0X lox_right;
Adafruit_VL53L0X lox_back;
Servo steeringServo;

// Trigonometric Constants (45 degrees)
const float SENSOR_ANGLE_RAD = 45.0 * (M_PI / 180.0);
const float SIN_45 = sin(SENSOR_ANGLE_RAD); // ~0.7071

// Servo Configuration
const int SERVO_CENTER = 90;     // Straight steering angle
const int SERVO_MIN = 45;        // Maximum left turn limit
const int SERVO_MAX = 135;       // Maximum right turn limit

// Control & Threshold Settings
const float DEADBAND_CM = 1.5;          // Deadband tolerance (in cm)
const float MAX_VALID_DIST_CM = 150.0;  // Max distance limit in cm

// PD Steering Parameters
float Kp = 0.8;
float Kd = 0.2;
float previousError = 0.0;
unsigned long lastTime = 0;

// Helper function to read distance in cm from Adafruit VL53L0X
float readSensorDistanceCM(Adafruit_VL53L0X &lox) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    // RangeStatus 4 indicates out-of-range / phase failure
    if (measure.RangeStatus != 4) {
        float distCm = measure.RangeMilliMeter / 10.0f;
        return constrain(distCm, 2.0f, MAX_VALID_DIST_CM);
    }
    return MAX_VALID_DIST_CM;
}

// Sequential initialization for multiple I2C sensors using XSHUT pins
void initVL53L0XSensors() {
    pinMode(XSHUT_LEFT_PIN, OUTPUT);
    pinMode(XSHUT_RIGHT_PIN, OUTPUT);
    pinMode(XSHUT_BACK_PIN, OUTPUT);
    // Reset both sensors by pulling XSHUT LOW
    digitalWrite(XSHUT_LEFT_PIN, LOW);
    digitalWrite(XSHUT_RIGHT_PIN, LOW);
    digitalWrite(XSHUT_BACK_PIN, LOW);  
    delay(10);  

    // 1. Initialize Left Sensor
    digitalWrite(XSHUT_LEFT_PIN, HIGH);
    delay(10);
    if (!lox_left.begin(L0X_LEFT_ADDR, false, &Wire)) {
        Serial.println("ERROR: Left VL53L0X failed to initialize!");
        while (true) delay(100);
    }

    // 2. Initialize Right Sensor
    digitalWrite(XSHUT_RIGHT_PIN, HIGH);
    delay(10);
    if (!lox_right.begin(L0X_RIGHT_ADDR, false, &Wire)) {
        Serial.println("ERROR: Right VL53L0X failed to initialize!");
        while (true) delay(100);
    }

    // // 3. Initialize Back Sensor
    // digitalWrite(XSHUT_BACK_PIN, HIGH);
    // delay(10);
    // if (!lox_back.begin(L0X_BACK_ADDR, false, &Wire)) {
    //     Serial.println("ERROR: Back VL53L0X failed to initialize!");
    //     while (true) delay(100);
    // }

    Serial.println("All VL53L0X sensors initialized successfully!");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("VL53L0X Self Reverse Parking System Starting...");

    // Initialize I2C Bus
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize Sensors
    initVL53L0XSensors();

    // Attach Steering Servo
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(SERVO_CENTER);
    
    delay(500);
}

void loop() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0f; // Time delta in seconds
    if (deltaTime <= 0.0f) deltaTime = 0.01f;

    // 1. Read Raw Distances from VL53L0X (converted from mm to cm)
    float rawLeftCm  = readSensorDistanceCM(lox_left);
    float rawRightCm = readSensorDistanceCM(lox_right);

    // 2. Trigonometric Decomposition to Lateral Distances (cm)
    float lateralLeft  = rawLeftCm * SIN_45;
    float lateralRight = rawRightCm * SIN_45;

    // 3. Compute Steering Error
    float error = lateralLeft - lateralRight;

    // 4. Deadband Implementation
    // If error is within +/- DEADBAND_CM, zero out error to prevent servo jitter
    if (fabs(error) < DEADBAND_CM) {
        error = 0.0f;
    }

    // 5. PD Steering Controller Calculation
    float derivative = (error - previousError) / deltaTime;
    float controlOutput = (Kp * error) + (Kd * derivative);

    // 6. Calculate Target Servo Angle (Reverse Ackerman Steering)
    int rawAngle = SERVO_CENTER + (int)controlOutput;
    int targetAngle = constrain(rawAngle, SERVO_MIN, SERVO_MAX);

    // Output to Servo
    steeringServo.write(targetAngle);

    // Update State Tracker
    previousError = error;
    lastTime = currentTime;

    // Telemetry Logging
    Serial.printf("Raw L: %.1f cm | Raw R: %.1f cm || Lat L: %.1f cm | Lat R: %.1f cm || Err: %.1f | Servo: %d deg\n", 
                  rawLeftCm, rawRightCm, lateralLeft, lateralRight, error, targetAngle);

    delay(40); // Loop execution rate (~25 Hz)
}