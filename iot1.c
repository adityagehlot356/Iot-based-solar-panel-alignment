/*******************************************************
 * Sun Tracker with Blynk (ESP32 DevKit V1)
 * Auto/Manual Servo + DHT11 + Voltage Sensor + LDRs
 *******************************************************/

#define BLYNK_TEMPLATE_ID "TMPL3vAIbyJGO"
#define BLYNK_TEMPLATE_NAME "Sun Tracking And Monitering"
#define BLYNK_AUTH_TOKEN "W6Xb0mutQX9FykVr6rsLZO5WOgGbZAk-"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include "DHT.h"

// ===== WiFi Credentials =====
char ssid[] = ".";           // Change this
char pass[] = "12345678";    // Change this

// ===== Pin Configuration =====
#define DHTPIN 4
#define DHTTYPE DHT11
#define LDR_LEFT 34
#define LDR_RIGHT 35
#define SERVO_PIN 13
#define VOLT_PIN 36  // use 36 or 34 for analog voltage sensor

// ===== Blynk Virtual Pins =====
#define VPIN_SERVO V0
#define VPIN_HUMIDITY V2
#define VPIN_TEMPERATURE V3
#define VPIN_VOLTAGE V4
#define VPIN_MANUAL_MODE V5

Servo servo;
DHT dht(DHTPIN, DHTTYPE);
BlynkTimer timer;

// ===== Variables =====
bool manualMode = false;
int servoPos = 90;
int threshold = 50;   // difference tolerance between LDRs
int offset = 600;     // LDR calibration offset

// ===== Functions =====
int getAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / 10;
}

// ===== BLYNK HANDLERS =====
BLYNK_WRITE(VPIN_SERVO) {
  if (manualMode) {
    servoPos = param.asInt();
    servo.write(servoPos);
    Serial.printf("Manual Servo Angle: %d°\n", servoPos);
  }
}

BLYNK_WRITE(VPIN_MANUAL_MODE) {
  manualMode = param.asInt();
  Serial.println(manualMode ? "Manual Mode ON" : "Auto Mode ON");
}

// ===== SENSOR FUNCTIONS =====
void sendSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float voltage = analogRead(VOLT_PIN) * (3.3 / 4095.0) * (25.0 / 5.0);

  if (!isnan(h) && !isnan(t)) {
    Blynk.virtualWrite(VPIN_HUMIDITY, h);
    Blynk.virtualWrite(VPIN_TEMPERATURE, t);
  }

  Blynk.virtualWrite(VPIN_VOLTAGE, voltage);

  Serial.printf("Temp: %.1f°C | Hum: %.1f%% | Volt: %.2fV\n", t, h, voltage);
}

// ===== AUTO TRACK FUNCTION =====
void autoTrack() {
  if (!manualMode) {
    int leftValue = getAverage(LDR_LEFT) - offset;
    int rightValue = getAverage(LDR_RIGHT);
    int diff = leftValue - rightValue;

    Serial.printf("Left: %d | Right: %d | Diff: %d | Servo: %d\n",
                  leftValue, rightValue, diff, servoPos);

    if (abs(diff) > threshold) {
      if (diff > 0 && servoPos < 180) servoPos++;
      else if (diff < 0 && servoPos > 0) servoPos--;
      servo.write(servoPos);
      Blynk.virtualWrite(VPIN_SERVO, servoPos);
      delay(15);
    }
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  servo.attach(SERVO_PIN);
  servo.write(servoPos);
  dht.begin();

  pinMode(LDR_LEFT, INPUT);
  pinMode(LDR_RIGHT, INPUT);
  pinMode(VOLT_PIN, INPUT);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Connecting to Blynk...");

  timer.setInterval(2000L, sendSensorData);
  timer.setInterval(500L, autoTrack);
}

// ===== LOOP =====
void loop() {
  Blynk.run();
  timer.run();
}