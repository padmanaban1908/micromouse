/**
 * ============================================================================
 * 🐭 MICROMOUSE: FORWARD MOVE & 90° RIGHT TURN CONTROLLER
 * ============================================================================
 * Features:
 *   - Drives smoothly FORWARD with active ToF sensor wall centering.
 *   - Continuous reading of all 5 ToF sensors (L, FL, FRONT, FR, R).
 *   - High-Precision 90° RIGHT TURN using MPU-6050 Gyro angle feedback.
 *   - Auto Turn: When front wall is detected (< 110mm), automatically
 *     executes a clean 90° Right Turn, then continues driving forward!
 *   - Button 1 (Pin 4) : TOGGLE Motor Drive ON / OFF.
 *   - Button 2 (Pin 13): TEST 90° RIGHT TURN immediately on command!
 *   - Live Serial Monitor telemetry at 115200 baud.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "tof.h"

// ============================================================================
// GLOBAL STATE & VARIABLES
// ============================================================================
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

float gyroZOffset = 0.0f;
float currentYawDeg = 0.0f;
float targetHeadingDeg = 0.0f; // Target heading for straight line tracking
unsigned long lastGyroMicros = 0;

Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t tofDistMm[NUM_TOF] = {999, 999, 999, 999, 999};

// Encoder ISRs with forward counting
void IRAM_ATTR isrEncoderLeft() {
#if INVERT_LEFT_ENCODER
    if (digitalRead(PIN_ENC_L_B)) encLeftTicks--; else encLeftTicks++;
#else
    if (digitalRead(PIN_ENC_L_B)) encLeftTicks++; else encLeftTicks--;
#endif
}

void IRAM_ATTR isrEncoderRight() {
#if INVERT_RIGHT_ENCODER
    if (digitalRead(PIN_ENC_R_B)) encRightTicks++; else encRightTicks--;
#else
    if (digitalRead(PIN_ENC_R_B)) encRightTicks++; else encRightTicks++;
#endif
}

// Drive State
bool motorsEnabled = false;       // Toggled by BTN1 (Pin 4)
int baseSpeed = SPEED_SEARCH_CRUISE; // 75 PWM (Safe 50% speed)
int turnSpeed = SPEED_SMOOTH_TURN;   // 80 PWM

unsigned long lastPrintTime = 0;
unsigned long lastBtnCheck = 0;

// ============================================================================
// 🔄 90° RIGHT TURN FUNCTION (MPU-6050 GYRO FEEDBACK)
// ============================================================================
void turnRight90() {
    Serial.println("\n↪️ [TURN] Executing 90° Right Turn...");
    stopMotors();
    delay(80); // Brief settle

    updateYaw();
    float startYaw = currentYawDeg;
    // Clockwise (Right) turn decreases yaw (Target = startYaw - 90.0)
    float targetChange = -90.0f;

    unsigned long turnStartTime = millis();

    // Rotate: Left wheel forward (+), Right wheel backward (-)
    while (millis() - turnStartTime < 1500) { // 1.5s safety timeout
        updateYaw();
        float turnedAngle = currentYawDeg - startYaw;

        // Stop when reached -88.0° to compensate for momentum
        if (turnedAngle <= -88.0f) {
            break;
        }

        // Taper speed for the last 15 degrees for pinpoint accuracy
        int currentSpeed = turnSpeed;
        if (turnedAngle <= -75.0f) {
            currentSpeed = (int)(turnSpeed * 0.70f); // Slow down to 70%
            if (currentSpeed < 50) currentSpeed = 50;
        }

        setMotorRaw(currentSpeed, -currentSpeed);
        delay(2);
    }

    stopMotors();
    delay(100);
    updateYaw();

    // Lock new target heading for straight driving in the new direction
    targetHeadingDeg = currentYawDeg;

    Serial.printf("✅ [TURN COMPLETE] Turned: %.1f° | New Heading: %.1f°\n\n",
                  currentYawDeg - startYaw, targetHeadingDeg);
}

// ============================================================================
// FORWARD DRIVE WITH SENSOR WALL CENTERING & FRONT OBSTACLE TURN
// ============================================================================
void driveForwardWithSensors() {
    // 1. Read all 5 ToF sensors & Gyro
    readAllToFSensors();
    updateYaw();

    uint16_t distL     = tofDistMm[SENS_LEFT];  // Left 90° (Pin 23)
    uint16_t distFL    = tofDistMm[SENS_FL];    // Front-Left 45° (Pin 19)
    uint16_t distFront = tofDistMm[SENS_FRONT]; // Front Center (Pin 18)
    uint16_t distFR    = tofDistMm[SENS_FR];    // Front-Right 45° (Pin 17)
    uint16_t distR     = tofDistMm[SENS_RIGHT]; // Right 90° (Pin 16)

    if (!motorsEnabled) {
        stopMotors();
        return;
    }

    // 2. Front Wall Detection (< 110mm ahead) -> EXECUTE 90° RIGHT TURN!
    if (distFront < 110 || distFL < 80 || distFR < 80) {
        Serial.printf("\n🛑 [WALL DETECTED] Front: %dmm, FL: %dmm, FR: %dmm -> Initiating 90° Right Turn!\n",
                      distFront, distFL, distFR);
        turnRight90();
        return;
    }

    // 3. Wall Centering Steering using Side ToF Sensors
    float steerCorrection = 0.0f;
    bool hasLeftWall  = (distL < WALL_SIDE_THRESHOLD_MM);  // < 130mm
    bool hasRightWall = (distR < WALL_SIDE_THRESHOLD_MM); // < 130mm

    if (hasLeftWall && hasRightWall) {
        // Both walls visible: Center in the corridor
        float wallError = (float)distL - (float)distR;
        steerCorrection = KP_WALL * wallError;
    } else if (hasLeftWall) {
        // Only left wall: Hold nominal distance (~84mm)
        float wallError = (float)distL - (float)NOMINAL_SIDE_DIST_MM;
        steerCorrection = KP_WALL * wallError * 1.2f;
    } else if (hasRightWall) {
        // Only right wall: Hold nominal distance (~84mm)
        float wallError = (float)NOMINAL_SIDE_DIST_MM - (float)distR;
        steerCorrection = KP_WALL * wallError * 1.2f;
    } else {
        // Open space: Hold current target heading using Gyro
        float headingError = targetHeadingDeg - currentYawDeg;
        float gyroRate = readGyroZRate();
        steerCorrection = KP_GYRO * headingError - KD_GYRO * gyroRate;
    }

    steerCorrection = constrain(steerCorrection, -35.0f, 35.0f);

    int pwmL = baseSpeed - (int)steerCorrection;
    int pwmR = baseSpeed + (int)steerCorrection;

    setMotorRaw(pwmL, pwmR);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================================");
    Serial.println("🐭 MICROMOUSE: FORWARD MOVE & 90° RIGHT TURN (GYRO)");
    Serial.println("========================================================");

    // 1. Initialize Buttons
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);

    // 2. Initialize Motors & Encoders
    initMotors();
    stopMotors();
    initEncoders();

    // 3. Initialize I2C Bus on SDA=21, SCL=22
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    // 4. Initialize MPU-6050 Gyro
    initMPU6050();
    targetHeadingDeg = currentYawDeg;

    // 5. Initialize 5x VL53L0X ToF Sensors
    initToFSensors();

    Serial.println("========================================================");
    Serial.println("✅ ALL SYSTEMS INITIALIZED AND READY!");
    Serial.println("👉 Press BTN1 (Pin 4)  : TOGGLE FORWARD DRIVE ON / OFF");
    Serial.println("👉 Press BTN2 (Pin 13) : TEST 90° RIGHT TURN NOW");
    Serial.println("🤖 Behavior: Drives forward, auto-turns 90° Right at front wall!");
    Serial.println("========================================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    // 1. Check Buttons
    if (millis() - lastBtnCheck > 250) {
        // Button 1 (Pin 4): Toggle Forward Drive
        if (digitalRead(PIN_BTN1) == LOW) {
            motorsEnabled = !motorsEnabled;
            if (motorsEnabled) {
                targetHeadingDeg = currentYawDeg;
                Serial.println("\n▶️ [FORWARD DRIVE ACTIVE] Moving forward with auto 90° right turn!");
            } else {
                stopMotors();
                Serial.println("\n⏸️ [MOTORS PAUSED] Sensor monitoring only.");
            }
            lastBtnCheck = millis();
        }

        // Button 2 (Pin 13): Test 90° Right Turn Immediately!
        if (digitalRead(PIN_BTN2) == LOW) {
            Serial.println("\n🔘 [BTN2 PRESSED] Manual 90° Right Turn Test Triggered!");
            turnRight90();
            lastBtnCheck = millis();
        }
    }

    // 2. Forward Drive & Automatic Wall Turn
    driveForwardWithSensors();

    // 3. Print Live Telemetry to Serial Monitor (Every 150ms)
    if (millis() - lastPrintTime >= 150) {
        lastPrintTime = millis();

        Serial.printf("[SENSORS] L:%3dmm | FL:%3dmm | FRONT:%3dmm | FR:%3dmm | R:%3dmm || Gyro:%5.1f° (Target:%5.1f°) || Status: ",
                      tofDistMm[SENS_LEFT],
                      tofDistMm[SENS_FL],
                      tofDistMm[SENS_FRONT],
                      tofDistMm[SENS_FR],
                      tofDistMm[SENS_RIGHT],
                      currentYawDeg,
                      targetHeadingDeg);

        if (motorsEnabled) {
            Serial.printf("🚀 FORWARD (PWM: %d)\n", baseSpeed);
        } else {
            Serial.println("⏸️ PAUSED (Press BTN1 to Start, BTN2 to Test Turn)");
        }
    }

    delay(5);
}
