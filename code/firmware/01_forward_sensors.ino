// ============================================================================
// 01_FORWARD_SENSORS: Simple Forward Driving with ToF Sensor Wall Centering
// ESP32 Arduino Core 2.x & 3.x compatible
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <esp_arduino_version.h>

// Motor Pins (TB6612)
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define BTN1 13   // Start / Pause (GPIO 13)
#define BTN2 4    // Emergency Stop / Zero (GPIO 4)

// I2C Bus & ToF
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

#define NUM_TOF 5
const int TOF_XSHUT_PINS[NUM_TOF]    = {23, 19, 18, 17, 16};
const uint8_t TOF_I2C_ADDRS[NUM_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34};

#define SENS_LEFT   0  // Left 90° (Pin 23)
#define SENS_FL     1  // Front-Left 45° (Pin 19)
#define SENS_FRONT  2  // Front Center 0° (Pin 18)
#define SENS_FR     3  // Front-Right 45° (Pin 17)
#define SENS_RIGHT  4  // Right 90° (Pin 16)

Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t distMm[NUM_TOF] = {999, 999, 999, 999, 999};

const int BASE_PWM = 75;
const float KP_WALL = 0.35f;
const int NOMINAL_DIST = 84; // mm to center in 168mm corridor

bool running = false;

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

void initToFSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    pinMode(TOF_XSHUT_PINS[i], OUTPUT);
    digitalWrite(TOF_XSHUT_PINS[i], LOW);
  }
  delay(20);
  for (int i = 0; i < NUM_TOF; i++) {
    digitalWrite(TOF_XSHUT_PINS[i], HIGH);
    delay(15);
    tofSensors[i].begin(TOF_I2C_ADDRS[i]);
  }
}

void readSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    tofSensors[i].rangingTest(&measure, false);
    distMm[i] = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  setupMotors();
  stopMotors();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  initToFSensors();

  Serial.println("=== SIMPLE FORWARD WITH SENSOR CENTERING ===");
  Serial.println("Press BTN1 (GPIO13) to Start/Stop. BTN2 (GPIO4) = E-Stop");
}

void loop() {
  if (digitalRead(BTN1) == LOW) {
    delay(50);
    if (digitalRead(BTN1) == LOW) {
      while (digitalRead(BTN1) == LOW) delay(10);
      running = !running;
      if (!running) stopMotors();
    }
  }
  if (digitalRead(BTN2) == LOW) {
    running = false;
    stopMotors();
  }

  readSensors();

  if (running) {
    // If front wall is closer than 100 mm, stop immediately
    if (distMm[SENS_FRONT] < 100 || distMm[SENS_FL] < 70 || distMm[SENS_FR] < 70) {
      stopMotors();
      Serial.println("⚠️ Front obstacle detected! Stopped.");
      return;
    }

    // Centering steering
    float steer = 0;
    bool hasL = distMm[SENS_LEFT] < 150;
    bool hasR = distMm[SENS_RIGHT] < 150;

    if (hasL && hasR) {
      steer = KP_WALL * ((float)distMm[SENS_LEFT] - (float)distMm[SENS_RIGHT]);
    } else if (hasL) {
      steer = KP_WALL * ((float)distMm[SENS_LEFT] - (float)NOMINAL_DIST);
    } else if (hasR) {
      steer = KP_WALL * ((float)NOMINAL_DIST - (float)distMm[SENS_RIGHT]);
    }

    steer = constrain(steer, -35.0f, 35.0f);
    setMotors(BASE_PWM - (int)steer, BASE_PWM + (int)steer);
  } else {
    stopMotors();
  }

  static unsigned long lastP = 0;
  if (millis() - lastP > 200) {
    lastP = millis();
    Serial.printf("L:%3d | FL:%3d | F:%3d | FR:%3d | R:%3d mm | %s\n",
                  distMm[SENS_LEFT], distMm[SENS_FL], distMm[SENS_FRONT], distMm[SENS_FR], distMm[SENS_RIGHT],
                  running ? "RUNNING" : "STOPPED");
  }
}
