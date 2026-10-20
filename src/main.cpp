// autoparking-car — ESP32-S3-DevKitC-1 (N16R8) stack verification
// Proves the full runtime stack compiles AND links:
//   VL53L0X x2 (XSHUT addressing) + ESP32Servo + WiFi + AsyncWebServer
// Placeholder firmware; real control firmware comes after the plan is signed off.

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

VL53L0X sensorL;
VL53L0X sensorR;
Servo steering;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// N16R8 pin map (octal PSRAM claims GPIO26-32; avoid strapping + USB)
constexpr int PIN_XSHUT_L = 15;
constexpr int PIN_XSHUT_R = 16;
constexpr int PIN_SERVO = 17;
constexpr int PIN_MOTOR_IN1 = 18;
constexpr int PIN_MOTOR_IN2 = 21;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  // SDA=8, SCL=9 (defaults, kept clear of XSHUT)

  // XSHUT sequencing: hold both low, then enable one at a time to assign addresses
  pinMode(PIN_XSHUT_L, OUTPUT);
  pinMode(PIN_XSHUT_R, OUTPUT);
  digitalWrite(PIN_XSHUT_L, LOW);
  digitalWrite(PIN_XSHUT_R, LOW);
  delay(20);
  digitalWrite(PIN_XSHUT_L, HIGH);
  delay(20);
  sensorL.setAddress(0x29);
  sensorL.init();
  sensorL.setMeasurementTimingBudget(50000);

  digitalWrite(PIN_XSHUT_R, HIGH);
  delay(20);
  sensorR.setAddress(0x2A);
  sensorR.init();
  sensorR.setMeasurementTimingBudget(50000);

  // Continuous mode so the 50 Hz PID loop never blocks on I2C
  sensorL.startContinuous(20);
  sensorR.startContinuous(20);

  steering.setPeriodHertz(50);
  steering.attach(PIN_SERVO, 500, 2500);
  steering.write(90);  // neutral

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  WiFi.begin("SSID", "PASSWORD");
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client,
                AwsEventType type, void*, uint8_t*, size_t len) {
    if (type == WS_EVT_CONNECT && len && client->canSend()) {
      client->text("{\"status\":\"ready\"}");
    }
  });
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "text/plain", "autoparking-car S3 N16R8 online");
  });
  server.begin();

  Serial.println("autoparking-car: full stack alive");
}

void loop() {
  Serial.printf("L=%dmm R=%dmm\n", sensorL.readRangeContinuousMillimeters(),
                sensorR.readRangeContinuousMillimeters());
  delay(100);
}