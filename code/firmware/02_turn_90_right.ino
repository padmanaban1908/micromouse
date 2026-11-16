// ============================================================================
// 02_TURN_90_RIGHT: Precision 90-Degree Right Pivot Turn with Gyro Feedback
// ESP32 Arduino Core 2.x & 3.x compatible
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>

#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define BTN1 13   // Press to execute 90° Right Turn (GPIO 13)
#define BTN2 4    // Emergency Stop / Zero Heading (GPIO 4)

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

const int turnSpeed = 80;
float gyroZOffset = 0.0f;
float currentYawDeg = 0.0f;
unsigned long lastGyroMicros = 0;

inline void pwmWriteLeft(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PWMA, val);
#else
  ledcWrite(0, val);
#endif
}

inline void pwmWriteRight(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PWMB, val);
#else
  ledcWrite(1, val);
#endif
}

void setupMotors() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);
#else
  ledcSetup(0, 5000, 8); ledcAttachPin(PWMA, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(PWMB, 1);
#endif
}

void setMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (left > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); pwmWriteLeft(left); }
  else if (left < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  pwmWriteLeft(-left); }
  else               { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  pwmWriteLeft(0); }

  if (right > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  pwmWriteRight(right); }
  else if (right < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); pwmWriteRight(-right); }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  pwmWriteRight(0); }
}

void stopMotors() { setMotors(0, 0); }

void initMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); // +/- 500 deg/s
  Wire.endTransmission(true);

  delay(100);
  Serial.print("Calibrating Gyro... keep robot still...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
    if (Wire.available() >= 2) sum += (Wire.read() << 8) | Wire.read();
    delay(2);
  }
  gyroZOffset = (float)sum / 500.0f;
  lastGyroMicros = micros();
  Serial.printf(" Done! Offset: %.2f\n", gyroZOffset);
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroMicros) * 1e-6f;
  lastGyroMicros = now;
  if (dt > 0.05f) dt = 0.005f;

  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
  if (Wire.available() >= 2) {
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    float rate = (float)(rawZ - gyroZOffset) / 65.5f;
    currentYawDeg += rate * dt;
  }
}

void turnRight90() {
  Serial.println("\n👉 Executing 90° RIGHT TURN...");
  stopMotors(); delay(100);
  updateYaw();
  float startYaw = currentYawDeg;
  unsigned long t0 = millis();

  while (millis() - t0 < 2000) {
    updateYaw();
    float turned = currentYawDeg - startYaw; // Decreases towards -90°
    if (turned <= -88.0f) break;
    int spd = (turned <= -75.0f) ? (int)(turnSpeed * 0.7f) : turnSpeed;
    if (spd < 50) spd = 50;
    setMotors(spd, -spd);
    delay(2);
  }
  stopMotors(); delay(100);
  updateYaw();
  Serial.printf("Turn completed! Final Heading: %.1f°\n", currentYawDeg);
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  setupMotors();
  stopMotors();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  initMPU6050();

  Serial.println("=== 90° RIGHT TURN TEST ===");
  Serial.println("Press BTN1 (GPIO13) to perform a 90° Right Turn");
  Serial.println("Press BTN2 (GPIO4) to reset heading to 0°");
}

void loop() {
  updateYaw();

  if (digitalRead(BTN1) == LOW) {
    delay(50);
    if (digitalRead(BTN1) == LOW) {
      while (digitalRead(BTN1) == LOW) delay(10);
      turnRight90();
    }
  }

  if (digitalRead(BTN2) == LOW) {
    delay(50);
    if (digitalRead(BTN2) == LOW) {
      while (digitalRead(BTN2) == LOW) delay(10);
      currentYawDeg = 0.0f;
      Serial.println("Heading reset to 0.0°");
    }
  }

  static unsigned long lastP = 0;
  if (millis() - lastP > 250) {
    lastP = millis();
    Serial.printf("Current Yaw: %6.1f°\n", currentYawDeg);
  }
}
