/**
 * ============================================================================
 * 🐭 AUTONOMOUS MICROMOUSE: LEFT-HAND RULE (LSRB) MAZE SOLVER
 * ============================================================================
 * Compatible with ESP32 Arduino Core 2.x and 3.x
 *
 * Hardware Specifications:
 *   - Microcontroller : ESP32 Dev Module (240 MHz)
 *   - Motor Driver    : TB6612FNG
 *       Left Motor    : PWMA=27, AIN1=14, AIN2=26 (FWD: AIN1=LOW, AIN2=HIGH)
 *       Right Motor   : PWMB=32, BIN1=25, BIN2=33 (FWD: BIN1=HIGH, BIN2=LOW)
 *   - Encoders        : 4x Quadrature, 4.7k pull-ups to 3.3V
 *       Left Encoder  : GPIO 34 (A), GPIO 35 (B) -> L_ENC_DIR = 1
 *       Right Encoder : GPIO 36 (A), GPIO 39 (B) -> R_ENC_DIR = -1
 *       Measured CPR  : ~2772.1 counts/rev
 *   - Buttons         : BTN1 = GPIO 13 (Start / Pause), BTN2 = GPIO 4 (Reset / Zero)
 *   - I2C Bus         : SDA = 21, SCL = 22 (400 kHz Fast I2C)
 *   - 5x ToF Sensors  : VL53L0X, XSHUT on pins 23(L), 19(FL), 18(F), 17(FR), 16(R)
 *   - IMU             : MPU-6050 (Direct I2C @ 0x68) for Z-axis Yaw gyro integration
 *
 * Left-Hand Rule (LSRB) Priority:
 *   1. Priority 1 (LEFT)     : If Left is open (> 150mm)  -> Turn 90° Left & Advance
 *   2. Priority 2 (STRAIGHT) : Else if Front is open      -> Drive Straight (Wall Centered)
 *   3. Priority 3 (RIGHT)    : Else if Right is open      -> Turn 90° Right & Advance
 *   4. Priority 4 (U-TURN)   : Else (All Blocked)         -> 180° Dead End U-Turn
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <esp_arduino_version.h>
#include "soc/gpio_struct.h"

// ============================================================================
// 1. PIN DEFINITIONS & CONSTANTS
// ============================================================================

// Motor Driver (TB6612)
#define PIN_PWMA 27
#define PIN_AIN1 14
#define PIN_AIN2 26

#define PIN_PWMB 32
#define PIN_BIN1 25
#define PIN_BIN2 33

// Quadrature Encoders (GPIO 34-39 are input-only, need external 4.7k pull-ups)
#define PIN_ENC_L_A 34
#define PIN_ENC_L_B 35
#define PIN_ENC_R_A 36
#define PIN_ENC_R_B 39

#define L_ENC_DIR  1
#define R_ENC_DIR -1

const float CPR_LEFT      = 2788.0f;
const float CPR_RIGHT     = 2788.0f;
const float WHEEL_DIAM_MM = 33.5f;
const float MM_PER_COUNT  = 3.14159265f * WHEEL_DIAM_MM / CPR_LEFT; // ~0.03775 mm per count

// Pushbuttons
#define PIN_BTN1 13   // Button 1: Start / Pause Maze Solver
#define PIN_BTN2 4    // Button 2: Reset Heading & Encoders

// I2C Pins
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// 5x VL53L0X Time-of-Flight Sensors
#define NUM_TOF 5
const int TOF_XSHUT_PINS[NUM_TOF]    = {23, 19, 18, 17, 16};
const uint8_t TOF_I2C_ADDRS[NUM_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34};

#define SENS_LEFT   0  // Left 90° (Pin 23)
#define SENS_FL     1  // Front-Left 45° (Pin 19)
#define SENS_FRONT  2  // Front Center 0° (Pin 18)
#define SENS_FR     3  // Front-Right 45° (Pin 17)
#define SENS_RIGHT  4  // Right 90° (Pin 16)

// PWM Settings
const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8; // 0-255

// Navigation Speeds (PWM values)
const int cruiseSpeed = 75; // 50% Cruise PWM
const int turnSpeed   = 80; // Pivot Turn PWM

// Sensor Decision Thresholds (in mm)
const uint16_t SIDE_OPEN_THRESHOLD_MM  = 150; // Side opening if distance > 150mm
const uint16_t FRONT_WALL_THRESHOLD_MM = 110; // Front blocked if distance < 110mm
const uint16_t NOMINAL_SIDE_DIST_MM    = 84;  // Center distance in 168mm corridor

// PID Control Gains
const float KP_WALL = 0.35f;
const float KP_GYRO = 2.50f;
const float KD_GYRO = 0.60f;

// ============================================================================
// 2. GLOBAL STATE
// ============================================================================
volatile int32_t encLeftTicks  = 0;
volatile int32_t encRightTicks = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

float gyroZOffset = 0.0f;
float currentYawDeg = 0.0f;
float targetHeadingDeg = 0.0f;
unsigned long lastGyroMicros = 0;

Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t tofDistMm[NUM_TOF] = {999, 999, 999, 999, 999};

bool solverActive = false;
unsigned long lastPrintTime = 0;
unsigned long lastBtnCheck = 0;

// Quadrature lookup: index = (prevAB << 2) | newAB
// 0 = no move, +1/-1 = one step, 2 = illegal
static const int8_t QDEC[16] = {
   0, -1,  1,  2,
   1,  0,  2, -1,
  -1,  2,  0,  1,
   2,  1, -1,  0
};

// Fast register read for GPIO 32-39
static inline uint8_t IRAM_ATTR readAB(uint8_t pinA, uint8_t pinB) {
  uint32_t in = GPIO.in1.val;
  return (((in >> (pinA - 32)) & 1) << 1) | ((in >> (pinB - 32)) & 1);
}

// Encoder ISRs
void IRAM_ATTR isrEncoderLeft() {
  uint8_t s = readAB(PIN_ENC_L_A, PIN_ENC_L_B);
  int8_t d = QDEC[(leftState << 2) | s];
  if (d != 2) encLeftTicks += d * L_ENC_DIR;
  leftState = s;
}

void IRAM_ATTR isrEncoderRight() {
  uint8_t s = readAB(PIN_ENC_R_A, PIN_ENC_R_B);
  int8_t d = QDEC[(rightState << 2) | s];
  if (d != 2) encRightTicks += d * R_ENC_DIR;
  rightState = s;
}

void initEncoders() {
  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);

  leftState = readAB(PIN_ENC_L_A, PIN_ENC_L_B);
  rightState = readAB(PIN_ENC_R_A, PIN_ENC_R_B);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncoderRight, CHANGE);
}

// ============================================================================
// 3. MOTOR DRIVER (ESP32 CORE 2.x & 3.x COMPATIBLE)
// ============================================================================
inline void pwmWriteLeft(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PIN_PWMA, val);
#else
  ledcWrite(pwmChannelLeft, val);
#endif
}

inline void pwmWriteRight(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PIN_PWMB, val);
#else
  ledcWrite(pwmChannelRight, val);
#endif
}

void setupMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcAttach(PIN_PWMA, pwmFreq, pwmResolution);
  ledcAttach(PIN_PWMB, pwmFreq, pwmResolution);
#else
  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
  ledcAttachPin(PIN_PWMA, pwmChannelLeft);
  ledcAttachPin(PIN_PWMB, pwmChannelRight);
#endif
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    pwmWriteLeft(speed);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    pwmWriteLeft(-speed);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    pwmWriteLeft(0);
  }
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    pwmWriteRight(speed);
  } else if (speed < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    pwmWriteRight(-speed);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    pwmWriteRight(0);
  }
}

void setMotorRaw(int leftPwm, int rightPwm) {
  setMotorLeft(leftPwm);
  setMotorRight(rightPwm);
}

void stopMotors() {
  setMotorLeft(0);
  setMotorRight(0);
}

// ============================================================================
// 4. MPU-6050 DIRECT I2C DRIVER (YAW INTEGRATION)
// ============================================================================
void initMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power Management 1
  Wire.write(0x00); // Wake up MPU-6050
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Gyro Config
  Wire.write(0x08); // +/- 500 deg/s range (65.5 LSB/(deg/s))
  Wire.endTransmission(true);

  delay(100);

  Serial.print("Calibrating Gyro... Keep robot still...");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
    if (Wire.available() >= 2) {
      sum += (Wire.read() << 8) | Wire.read();
    }
    delay(2);
  }
  gyroZOffset = (float)sum / 500.0f;
  lastGyroMicros = micros();
  Serial.printf(" Done! Bias: %.2f\n", gyroZOffset);
}

float readGyroZRate() {
  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)2, (uint8_t)true);
  if (Wire.available() >= 2) {
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    return (float)(rawZ - gyroZOffset) / 65.5f;
  }
  return 0.0f;
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroMicros) / 1000000.0f;
  lastGyroMicros = now;
  if (dt > 0.05f) dt = 0.005f;
  currentYawDeg += readGyroZRate() * dt;
}

// ============================================================================
// 5. 5x VL53L0X TOF SENSOR DRIVERS
// ============================================================================
void initToFSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    pinMode(TOF_XSHUT_PINS[i], OUTPUT);
    digitalWrite(TOF_XSHUT_PINS[i], LOW);
  }
  delay(20);

  for (int i = 0; i < NUM_TOF; i++) {
    digitalWrite(TOF_XSHUT_PINS[i], HIGH);
    delay(15);
    if (tofSensors[i].begin(TOF_I2C_ADDRS[i])) {
      Serial.printf("✅ ToF %d online at 0x%02X\n", i, TOF_I2C_ADDRS[i]);
    } else {
      Serial.printf("⚠️ ToF %d failed!\n", i);
    }
    delay(10);
  }
}

void readAllToFSensors() {
  for (int i = 0; i < NUM_TOF; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    tofSensors[i].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      tofDistMm[i] = measure.RangeMilliMeter;
    } else {
      tofDistMm[i] = 999;
    }
  }
}

// ============================================================================
// 6. CLOSED-LOOP GYRO TURNING ROUTINES
// ============================================================================

// 90° Turn LEFT
void turnLeft90() {
  Serial.println("\n👈 [LSRB] Priority 1: TURN LEFT 90°");
  stopMotors(); delay(80);
  updateYaw();
  float startYaw = currentYawDeg;
  unsigned long turnStartTime = millis();

  while (millis() - turnStartTime < 1500) {
    updateYaw();
    float turned = currentYawDeg - startYaw; // Increases toward +90°
    if (turned >= 88.0f) break;
    int speed = (turned >= 75.0f) ? (int)(turnSpeed * 0.70f) : turnSpeed;
    if (speed < 50) speed = 50;
    setMotorRaw(-speed, speed);
    delay(2);
  }
  stopMotors(); delay(100);
  updateYaw();
  targetHeadingDeg = currentYawDeg;
}

// 90° Turn RIGHT
void turnRight90() {
  Serial.println("\n👉 [LSRB] Priority 3: TURN RIGHT 90°");
  stopMotors(); delay(80);
  updateYaw();
  float startYaw = currentYawDeg;
  unsigned long turnStartTime = millis();

  while (millis() - turnStartTime < 1500) {
    updateYaw();
    float turned = currentYawDeg - startYaw; // Decreases toward -90°
    if (turned <= -88.0f) break;
    int speed = (turned <= -75.0f) ? (int)(turnSpeed * 0.70f) : turnSpeed;
    if (speed < 50) speed = 50;
    setMotorRaw(speed, -speed);
    delay(2);
  }
  stopMotors(); delay(100);
  updateYaw();
  targetHeadingDeg = currentYawDeg;
}

// 180° Dead End U-TURN
void turnUTurn180() {
  Serial.println("\n🔄 [LSRB] Priority 4: DEAD END -> 180° U-TURN");
  stopMotors(); delay(80);
  updateYaw();
  float startYaw = currentYawDeg;
  unsigned long turnStartTime = millis();

  while (millis() - turnStartTime < 2500) {
    updateYaw();
    float turned = currentYawDeg - startYaw;
    if (turned <= -176.0f || turned >= 176.0f) break;
    setMotorRaw(turnSpeed, -turnSpeed);
    delay(2);
  }
  stopMotors(); delay(100);
  updateYaw();
  targetHeadingDeg = currentYawDeg;
}

// Drive straight a fixed duration to pass junction axle
void inchForward(int durationMs) {
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)durationMs) {
    updateYaw();
    float headingError = targetHeadingDeg - currentYawDeg;
    float steer = KP_GYRO * headingError;
    setMotorRaw(cruiseSpeed - (int)steer, cruiseSpeed + (int)steer);
    delay(2);
  }
  stopMotors();
  delay(40);
}

// ============================================================================
// 7. LEFT-HAND RULE (LSRB) DECISION ENGINE
// ============================================================================
void executeLeftHandRuleStep() {
  readAllToFSensors();
  updateYaw();

  uint16_t distL     = tofDistMm[SENS_LEFT];
  uint16_t distFL    = tofDistMm[SENS_FL];
  uint16_t distFront = tofDistMm[SENS_FRONT];
  uint16_t distFR    = tofDistMm[SENS_FR];
  uint16_t distR     = tofDistMm[SENS_RIGHT];

  bool leftIsOpen  = (distL > SIDE_OPEN_THRESHOLD_MM);
  bool frontIsOpen = (distFront > FRONT_WALL_THRESHOLD_MM) && (distFL > 80) && (distFR > 80);
  bool rightIsOpen = (distR > SIDE_OPEN_THRESHOLD_MM);

  // PRIORITY 1: LEFT IS OPEN -> Turn 90° Left and advance
  if (leftIsOpen) {
    Serial.printf("[LSRB] Left is OPEN (%dmm) -> Taking Left Turn!\n", distL);
    inchForward(180);
    turnLeft90();
    inchForward(250);
    return;
  }

  // PRIORITY 2: STRAIGHT IS OPEN -> Drive straight with wall centering
  if (frontIsOpen) {
    float steer = 0.0f;
    bool hasL = (distL < SIDE_OPEN_THRESHOLD_MM);
    bool hasR = (distR < SIDE_OPEN_THRESHOLD_MM);

    if (hasL && hasR) {
      steer = KP_WALL * ((float)distL - (float)distR);
    } else if (hasL) {
      steer = KP_WALL * ((float)distL - (float)NOMINAL_SIDE_DIST_MM) * 1.2f;
    } else if (hasR) {
      steer = KP_WALL * ((float)NOMINAL_SIDE_DIST_MM - (float)distR) * 1.2f;
    } else {
      steer = KP_GYRO * (targetHeadingDeg - currentYawDeg) - KD_GYRO * readGyroZRate();
    }
    steer = constrain(steer, -35.0f, 35.0f);
    setMotorRaw(cruiseSpeed - (int)steer, cruiseSpeed + (int)steer);
    return;
  }

  // PRIORITY 3: RIGHT IS OPEN -> Turn 90° Right and advance
  if (rightIsOpen) {
    Serial.printf("[LSRB] Left & Front blocked, Right is OPEN (%dmm) -> Turn Right!\n", distR);
    turnRight90();
    inchForward(150);
    return;
  }

  // PRIORITY 4: DEAD END -> Turn 180° around
  Serial.println("[LSRB] All paths blocked -> Dead End 180° U-Turn!");
  turnUTurn180();
  inchForward(150);
}

// ============================================================================
// 8. SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);

  setupMotors();
  stopMotors();
  initEncoders(); // Hardware 4x Quadrature Encoders Initialized!

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

  initMPU6050();
  targetHeadingDeg = currentYawDeg;

  initToFSensors();

  Serial.println("\n========================================================");
  Serial.println("🐭 MICROMOUSE: LEFT-HAND RULE (LSRB) SOLVER READY!");
  Serial.println("👉 Press BTN1 (GPIO 13) : START / PAUSE Left-Hand Solver");
  Serial.println("👉 Press BTN2 (GPIO 4)  : Reset Yaw Heading & Encoders to 0");
  Serial.println("🧭 Priority Order: LEFT -> STRAIGHT -> RIGHT -> BACK");
  Serial.println("========================================================\n");
}

void loop() {
  // Check Buttons
  if (millis() - lastBtnCheck > 250) {
    // BTN1 (GPIO 13): Toggle Start / Pause
    if (digitalRead(PIN_BTN1) == LOW) {
      solverActive = !solverActive;
      if (solverActive) {
        targetHeadingDeg = currentYawDeg;
        Serial.println("\n▶️ [SOLVER STARTED] Exploring via Left-Hand Rule...");
      } else {
        stopMotors();
        Serial.println("\n⏸️ [SOLVER PAUSED] Motors stopped.");
      }
      lastBtnCheck = millis();
    }

    // BTN2 (GPIO 4): Reset Heading & Encoders
    if (digitalRead(PIN_BTN2) == LOW) {
      currentYawDeg = 0.0f;
      targetHeadingDeg = 0.0f;
      noInterrupts();
      encLeftTicks = 0;
      encRightTicks = 0;
      interrupts();
      Serial.println("\n🔄 [RESET] Heading and Encoders zeroed to 0!");
      lastBtnCheck = millis();
    }
  }

  // Execute Left-Hand Rule navigation when active
  if (solverActive) {
    executeLeftHandRuleStep();
  } else {
    stopMotors();
    readAllToFSensors();
    updateYaw();
  }

  // Print Telemetry Stream every 200ms
  if (millis() - lastPrintTime >= 200) {
    lastPrintTime = millis();
    Serial.printf("[LSRB] L:%3dmm | FL:%3dmm | FRONT:%3dmm | FR:%3dmm | R:%3dmm || Gyro:%5.1f° || %s\n",
                  tofDistMm[SENS_LEFT],
                  tofDistMm[SENS_FL],
                  tofDistMm[SENS_FRONT],
                  tofDistMm[SENS_FR],
                  tofDistMm[SENS_RIGHT],
                  currentYawDeg,
                  solverActive ? "RUNNING" : "PAUSED");
  }

  delay(4);
}
