// autoparking-car: ESP32-S3-DevKitC-1 (N16R8) stack verification
// proves the full runtime stack compiles and links:
//   VL53L0X x2 (XSHUT addressing) + ESP32Servo + WiFi + AsyncWebServer
// this is placeholder firmware. the real control firmware comes after the plan
// is signed off.

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

// N16R8 pin map. octal PSRAM claims GPIO 26 to 32, so those stay clear, along
// with the USB pins and the strapping pins sampled at boot. ---
constexpr int PIN_XSHUT_L = 15;
constexpr int PIN_XSHUT_R = 16;
constexpr int PIN_SERVO = 17;
constexpr int PIN_MOTOR_IN1 = 18;
constexpr int PIN_MOTOR_IN2 = 21;
// end

void setup() {
  Serial.begin(115200);
  // SDA is 8 and SCL is 9, both kept clear of the XSHUT lines. ---
  Wire.begin(8, 9);
  // end

  // XSHUT sequencing. hold all the sensors low, then enable one at a time and
  // give it its address. ---
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
  // end

  // continuous mode, so the control loop never blocks on I2C. ---
  sensorL.startContinuous(20);
  sensorR.startContinuous(20);

  steering.setPeriodHertz(50);
  steering.attach(PIN_SERVO, 500, 2400);
  steering.write(90);  // neutral

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  // end

  // join the network and report the address on the serial console. ---
  WiFi.begin("SSID", "PASSWORD");
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  // end

  // web socket and a plain status page. ---
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
  // end

  Serial.println("autoparking-car: full stack alive");
}

void loop() {
  // print both sensor readings so the I2C path can be checked by eye. ---
  Serial.printf("L=%dmm R=%dmm\n", sensorL.readRangeContinuousMillimeters(),
                sensorR.readRangeContinuousMillimeters());
  delay(100);
  // end
}