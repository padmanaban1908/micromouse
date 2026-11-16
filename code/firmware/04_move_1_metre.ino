// ============================================================================
//  MOVE 1 METRE  (ESP32 Arduino core 3.x & 2.x compatible)
//  BTN1 (GPIO13) -> go FORWARD 1000 mm
//  BTN2 (GPIO4)  -> go BACKWARD 1000 mm
//  Pressing any button during a run = emergency stop.
//  Wheel dia 33.5 mm, 2788 counts per revolution (4x decoding)
// ============================================================================
#include <Arduino.h>
#include "soc/gpio_struct.h"
#include <esp_arduino_version.h>

// ---------------- PINS (same as your calibration program) ----------------
#define PWMA 27
#define AIN1 14
#define AIN2 26
#define PWMB 32
#define BIN1 25
#define BIN2 33

#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 36
#define R_ENC_B 39
#define L_ENC_DIR  1
#define R_ENC_DIR -1

#define BTN1 13      // forward
#define BTN2 4       // backward

// ---------------- SETTINGS ----------------
const float WHEEL_DIAM_MM = 33.5f;
const float CPR           = 2788.0f;
const float MM_PER_COUNT  = PI * WHEEL_DIAM_MM / CPR; // = 0.03775 mm per count

const float MOVE_DIST_MM  = 1000.0f;   // 1 metre
const float MOVE_SPEED    = 200.0f;    // mm/s (raise later)
const float ACCEL         = 800.0f;    // mm/s^2

// Speed controller gains
const float KFF = 0.45f;               // PWM per mm/s
const float KP  = 0.25f;
const float KI  = 3.0f;
const float K_STRAIGHT = 2.0f;         // mm/s correction per mm of L/R difference

const int pwmFreq = 5000;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int pwmResolution = 8;
#define TICK_US 2000UL

// ---------------- ENCODERS ----------------
volatile int32_t leftCount = 0, rightCount = 0;
volatile uint8_t leftState = 0, rightState = 0;

static const int8_t QDEC[16] = {
   0, -1,  1,  2,
   1,  0,  2, -1,
  -1,  2,  0,  1,
   2,  1, -1,  0
};

static inline uint8_t IRAM_ATTR readAB(uint8_t pinA, uint8_t pinB) {
  uint32_t in = GPIO.in1.val;
  return (((in >> (pinA - 32)) & 1) << 1) | ((in >> (pinB - 32)) & 1);
}

void IRAM_ATTR leftEncoderISR() {
  uint8_t s = readAB(L_ENC_A, L_ENC_B);
  int8_t d = QDEC[(leftState << 2) | s];
  if (d != 2) leftCount += d * L_ENC_DIR;
  leftState = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = readAB(R_ENC_A, R_ENC_B);
  int8_t d = QDEC[(rightState << 2) | s];
  if (d != 2) rightCount += d * R_ENC_DIR;
  rightState = s;
}

void readCounts(int32_t &l, int32_t &r) {
  noInterrupts(); l = leftCount; r = rightCount; interrupts();
}

// ---------------- MOTORS ----------------
// Dual compatibility helper for ESP32 Arduino Core 2.x and 3.x
inline void pwmWriteLeft(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PWMA, val);
#else
  ledcWrite(pwmChannelLeft, val);
#endif
}

inline void pwmWriteRight(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcWrite(PWMB, val);
#else
  ledcWrite(pwmChannelRight, val);
#endif
}

void setupMotors() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  ledcAttach(PWMA, pwmFreq, pwmResolution);
  ledcAttach(PWMB, pwmFreq, pwmResolution);
#else
  ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
  ledcAttachPin(PWMA, pwmChannelLeft);
  ledcAttachPin(PWMB, pwmChannelRight);
#endif
}

void setMotorLeft(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); pwmWriteLeft(speed); }
  else if (speed < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  pwmWriteLeft(-speed); }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  pwmWriteLeft(0); }
}

void setMotorRight(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  pwmWriteRight(speed); }
  else if (speed < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); pwmWriteRight(-speed); }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  pwmWriteRight(0); }
}

void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); pwmWriteLeft(255);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); pwmWriteRight(255);
}

void stopMotors() { setMotorLeft(0); setMotorRight(0); }

// ---------------- SPEED CONTROL ----------------
// target / meas are "forward-normalised" speeds (always positive when moving as commanded)
int wheelPWM(float target, float meas, float &integ, float dt) {
  float err = target - meas;
  integ += KI * err * dt;
  integ = constrain(integ, -90.0f, 90.0f);
  float out = KFF * target + KP * err + integ;
  return (int)constrain(out, -255.0f, 255.0f);
}

void waitRelease(int pin) {
  while (digitalRead(pin) == LOW) delay(10);
  delay(50);
}

// dir = +1 forward, -1 backward
void runMove(int dir) {
  Serial.printf("\n>>> Moving %s %.0f mm\n", dir > 0 ? "FORWARD" : "BACKWARD", MOVE_DIST_MM);

  int32_t lastL, lastR;
  readCounts(lastL, lastR);

  float progL = 0, progR = 0;      // mm travelled in the commanded direction
  float vL = 0, vR = 0;            // measured speed, normalised (mm/s)
  float vCmd = 0;
  float iL = 0, iR = 0;
  uint16_t stall = 0;
  uint32_t lastUs = micros();
  uint32_t startMs = millis();
  uint32_t printMs = 0;
  bool aborted = false;

  while (true) {
    uint32_t now;
    do { now = micros(); } while ((uint32_t)(now - lastUs) < TICK_US);
    float dt = (now - lastUs) * 1e-6f;
    lastUs = now;

    if (digitalRead(BTN1) == LOW || digitalRead(BTN2) == LOW) { Serial.println("*** EMERGENCY STOP ***"); aborted = true; break; }
    if (millis() - startMs > 15000) { Serial.println("*** TIMEOUT ***"); aborted = true; break; }

    int32_t l, r;
    readCounts(l, r);
    float dL = (l - lastL) * MM_PER_COUNT * dir;
    float dR = (r - lastR) * MM_PER_COUNT * dir;
    lastL = l; lastR = r;
    progL += dL; progR += dR;
    vL += 0.35f * (dL / dt - vL);
    vR += 0.35f * (dR / dt - vR);

    float prog = 0.5f * (progL + progR);
    float remaining = MOVE_DIST_MM - prog;
    if (remaining <= 0.0f) break;

    // speed profile: accelerate, cruise, brake smoothly to the target
    float vTarget = MOVE_SPEED;
    float vStop = sqrtf(2.0f * ACCEL * remaining) + 20.0f;
    if (vStop < vTarget) vTarget = vStop;
    if (vTarget > vCmd) vCmd = fminf(vCmd + ACCEL * dt, vTarget);
    else                vCmd = fmaxf(vCmd - ACCEL * dt, vTarget);

    // keep both wheels travelling equal distance -> straight line
    float diff = progL - progR;
    float tL = vCmd - K_STRAIGHT * diff;
    float tR = vCmd + K_STRAIGHT * diff;

    setMotorLeft (dir * wheelPWM(tL, vL, iL, dt));
    setMotorRight(dir * wheelPWM(tR, vR, iR, dt));

    // safety: commanded to move but encoders do not show motion in that direction
    if (vCmd > 100.0f && 0.5f * (vL + vR) < 20.0f) stall++; else stall = 0;
    if (stall > 500) {
      Serial.println("*** STOP: not moving in the commanded direction. Check encoder DIR signs / wiring. ***");
      aborted = true;
      break;
    }

    if (millis() - printMs > 200) {
      printMs = millis();
      Serial.printf("dist %.0f mm | L %.0f R %.0f mm/s | L-R %.1f mm\n", prog, vL, vR, diff);
    }
  }

  brakeMotors();
  delay(300);
  stopMotors();

  if (!aborted)
    Serial.printf("DONE. L %.1f mm, R %.1f mm. Measure the real distance with a ruler.\n", progL, progR);
  Serial.println("Press BTN1 = forward, BTN2 = backward.");
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  setupMotors();
  stopMotors();

  pinMode(L_ENC_A, INPUT); pinMode(L_ENC_B, INPUT);
  pinMode(R_ENC_A, INPUT); pinMode(R_ENC_B, INPUT);
  leftState  = readAB(L_ENC_A, L_ENC_B);
  rightState = readAB(R_ENC_A, R_ENC_B);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_ENC_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_B), rightEncoderISR, CHANGE);

  Serial.println("\n=== MOVE 1 METRE ===");
  Serial.println("BTN1 = FORWARD 1 m | BTN2 = BACKWARD 1 m | any button during run = STOP");
}

void loop() {
  if (digitalRead(BTN1) == LOW) {
    delay(50);
    if (digitalRead(BTN1) == LOW) { waitRelease(BTN1); delay(500); runMove(+1); }
  } else if (digitalRead(BTN2) == LOW) {
    delay(50);
    if (digitalRead(BTN2) == LOW) { waitRelease(BTN2); delay(500); runMove(-1); }
  }
}
