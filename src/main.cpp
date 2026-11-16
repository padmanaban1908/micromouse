/**
 * ============================================================================
 * 🐭 AUTONOMOUS MICROMOUSE: LEFT-HAND RULE (LSRB) MAZE SOLVER
 * ============================================================================
 * Hardware Pin Specifications:
 *   - Left Motor   : PWMA=27, AIN1=14, AIN2=26
 *   - Right Motor  : PWMB=32, BIN1=25, BIN2=33 (Inverted for forward symmetry)
 *   - Encoders     : Left=(36, 39), Right=(34, 35) + 4.7k external pull-ups
 *   - I2C Bus      : SDA=21, SCL=22 (400 kHz Fast I2C)
 *   - 5x ToF XSHUT : 23, 19, 18, 17, 16 (VL53L0X sensors)
 *   - IMU          : MPU-6050 (Direct I2C register access @ 0x68)
 *   - Pushbuttons  : BTN1=4 (Start / Pause), BTN2=13 (Reset/Test)
 *
 * Algorithm (Left-Hand on Wall Priority):
 *   1. Priority 1 (LEFT)     : If Left is open (> 150mm) -> Turn 90° Left & Go
 *   2. Priority 2 (STRAIGHT) : Else if Front is open (> 110mm) -> Drive Straight
 *   3. Priority 3 (RIGHT)    : Else if Right is open (> 150mm) -> Turn 90° Right & Go
 *   4. Priority 4 (BACK)     : Else (Dead End) -> 180° U-Turn
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
volatile int32_t encLeftTicks  = 0;
volatile int32_t encRightTicks = 0;
volatile uint8_t leftEncState = 0;
volatile uint8_t rightEncState = 0;

float gyroZOffset = 0.0f;
float currentYawDeg = 0.0f;
float targetHeadingDeg = 0.0f;
unsigned long lastGyroMicros = 0;

Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t tofDistMm[NUM_TOF] = {999, 999, 999, 999, 999};

// Encoder ISRs (4x Quadrature Decoding)
void IRAM_ATTR isrEncoderLeft() {
    uint8_t s = readEncAB(PIN_ENC_L_A, PIN_ENC_L_B);
    int8_t d = QDEC_TABLE[(leftEncState << 2) | s];
    if (d != 2) encLeftTicks += d * L_ENC_DIR;
    leftEncState = s;
}

void IRAM_ATTR isrEncoderRight() {
    uint8_t s = readEncAB(PIN_ENC_R_A, PIN_ENC_R_B);
    int8_t d = QDEC_TABLE[(rightEncState << 2) | s];
    if (d != 2) encRightTicks += d * R_ENC_DIR;
    rightEncState = s;
}

// Drive State
bool solverActive = false;           // Toggled by BTN1 (Pin 4)
int cruiseSpeed = SPEED_SEARCH_CRUISE; // 75 PWM (Safe 50% speed)
int turnSpeed   = SPEED_SMOOTH_TURN;   // 80 PWM

// Thresholds for Maze Decision
const uint16_t SIDE_OPEN_THRESHOLD_MM  = 150; // Side opening if distance > 150mm
const uint16_t FRONT_WALL_THRESHOLD_MM = 110; // Front blocked if distance < 110mm

unsigned long lastPrintTime = 0;
unsigned long lastBtnCheck = 0;

// ============================================================================
// 🔄 PRECISION GYRO TURN ROUTINES
// ============================================================================

/**
 * Turn 90° LEFT (Counter-Clockwise)
 */
void turnLeft90() {
    Serial.println("\n👈 [LSRB] Action: TURN LEFT 90°");
    stopMotors();
    delay(80);

    updateYaw();
    float startYaw = currentYawDeg;

    unsigned long turnStartTime = millis();

    // Left wheel backward (-), Right wheel forward (+)
    while (millis() - turnStartTime < 1500) {
        updateYaw();
        float turnedAngle = currentYawDeg - startYaw; // Increases toward +90°

        if (turnedAngle >= 88.0f) {
            break;
        }

        int currentSpeed = turnSpeed;
        if (turnedAngle >= 75.0f) {
            currentSpeed = (int)(turnSpeed * 0.70f);
            if (currentSpeed < 50) currentSpeed = 50;
        }

        setMotorRaw(-currentSpeed, currentSpeed);
        delay(2);
    }

    stopMotors();
    delay(100);
    updateYaw();
    targetHeadingDeg = currentYawDeg;
    Serial.printf("✅ [TURN LEFT DONE] Angle: %.1f° | Heading: %.1f°\n\n",
                  currentYawDeg - startYaw, targetHeadingDeg);
}

/**
 * Turn 90° RIGHT (Clockwise)
 */
void turnRight90() {
    Serial.println("\n👉 [LSRB] Action: TURN RIGHT 90°");
    stopMotors();
    delay(80);

    updateYaw();
    float startYaw = currentYawDeg;

    unsigned long turnStartTime = millis();

    // Left wheel forward (+), Right wheel backward (-)
    while (millis() - turnStartTime < 1500) {
        updateYaw();
        float turnedAngle = currentYawDeg - startYaw; // Decreases toward -90°

        if (turnedAngle <= -88.0f) {
            break;
        }

        int currentSpeed = turnSpeed;
        if (turnedAngle <= -75.0f) {
            currentSpeed = (int)(turnSpeed * 0.70f);
            if (currentSpeed < 50) currentSpeed = 50;
        }

        setMotorRaw(currentSpeed, -currentSpeed);
        delay(2);
    }

    stopMotors();
    delay(100);
    updateYaw();
    targetHeadingDeg = currentYawDeg;
    Serial.printf("✅ [TURN RIGHT DONE] Angle: %.1f° | Heading: %.1f°\n\n",
                  currentYawDeg - startYaw, targetHeadingDeg);
}

/**
 * Turn 180° U-TURN (Dead End)
 */
void turnUTurn180() {
    Serial.println("\n🔄 [LSRB] Action: DEAD END -> 180° U-TURN");
    stopMotors();
    delay(80);

    updateYaw();
    float startYaw = currentYawDeg;

    unsigned long turnStartTime = millis();

    while (millis() - turnStartTime < 2500) {
        updateYaw();
        float turnedAngle = currentYawDeg - startYaw;

        if (turnedAngle <= -176.0f || turnedAngle >= 176.0f) {
            break;
        }

        setMotorRaw(turnSpeed, -turnSpeed);
        delay(2);
    }

    stopMotors();
    delay(100);
    updateYaw();
    targetHeadingDeg = currentYawDeg;
    Serial.printf("✅ [U-TURN DONE] Heading: %.1f°\n\n", targetHeadingDeg);
}

// ============================================================================
// 📏 INCH FORWARD (Pass Junction Axle)
// ============================================================================
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
// 🧭 LEFT-HAND RULE DECISION ENGINE (LSRB)
// ============================================================================
void executeLeftHandRuleStep() {
    // 1. Read all sensors
    readAllToFSensors();
    updateYaw();

    uint16_t distL     = tofDistMm[SENS_LEFT];  // Pin 23
    uint16_t distFL    = tofDistMm[SENS_FL];    // Pin 19
    uint16_t distFront = tofDistMm[SENS_FRONT]; // Pin 18
    uint16_t distFR    = tofDistMm[SENS_FR];    // Pin 17
    uint16_t distR     = tofDistMm[SENS_RIGHT]; // Pin 16

    bool leftIsOpen  = (distL > SIDE_OPEN_THRESHOLD_MM);
    bool frontIsOpen = (distFront > FRONT_WALL_THRESHOLD_MM) && (distFL > 80) && (distFR > 80);
    bool rightIsOpen = (distR > SIDE_OPEN_THRESHOLD_MM);

    // ========================================================================
    // PRIORITY 1: LEFT TURN (Always take Left if opening exists)
    // ========================================================================
    if (leftIsOpen) {
        Serial.printf("[DECISION] Left is OPEN (%dmm) -> Taking Left Turn!\n", distL);
        // Inch forward slightly so axle reaches corner pivot center
        inchForward(180);
        turnLeft90();
        // Drive forward into the new branch
        inchForward(250);
        return;
    }

    // ========================================================================
    // PRIORITY 2: STRAIGHT (Go straight if Left is blocked but path is open)
    // ========================================================================
    if (frontIsOpen) {
        // Active wall centering
        float steerCorrection = 0.0f;
        bool hasLeftWall  = (distL < SIDE_OPEN_THRESHOLD_MM);
        bool hasRightWall = (distR < SIDE_OPEN_THRESHOLD_MM);

        if (hasLeftWall && hasRightWall) {
            float wallError = (float)distL - (float)distR;
            steerCorrection = KP_WALL * wallError;
        } else if (hasLeftWall) {
            float wallError = (float)distL - (float)NOMINAL_SIDE_DIST_MM;
            steerCorrection = KP_WALL * wallError * 1.2f;
        } else if (hasRightWall) {
            float wallError = (float)NOMINAL_SIDE_DIST_MM - (float)distR;
            steerCorrection = KP_WALL * wallError * 1.2f;
        } else {
            float headingError = targetHeadingDeg - currentYawDeg;
            float gyroRate = readGyroZRate();
            steerCorrection = KP_GYRO * headingError - KD_GYRO * gyroRate;
        }

        steerCorrection = constrain(steerCorrection, -35.0f, 35.0f);
        setMotorRaw(cruiseSpeed - (int)steerCorrection, cruiseSpeed + (int)steerCorrection);
        return;
    }

    // ========================================================================
    // PRIORITY 3: RIGHT TURN (Left and Straight blocked, Right is open)
    // ========================================================================
    if (rightIsOpen) {
        Serial.printf("[DECISION] Left & Front blocked, Right is OPEN (%dmm) -> Turning Right!\n", distR);
        turnRight90();
        inchForward(150);
        return;
    }

    // ========================================================================
    // PRIORITY 4: DEAD END (Left, Front, and Right are all blocked!)
    // ========================================================================
    Serial.println("[DECISION] All directions blocked -> DEAD END detected!");
    turnUTurn180();
    inchForward(150);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================================");
    Serial.println("🐭 AUTONOMOUS MICROMOUSE: LEFT-HAND RULE (LSRB) SOLVER");
    Serial.println("========================================================");

    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);

    initMotors();
    stopMotors();
    initEncoders();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    initMPU6050();
    targetHeadingDeg = currentYawDeg;

    initToFSensors();

    Serial.println("========================================================");
    Serial.println("✅ SYSTEM READY!");
    Serial.println("👉 Press BTN1 (GPIO 13) : START / PAUSE Left-Hand Maze Solver");
    Serial.println("👉 Press BTN2 (GPIO 4)  : Reset Yaw Heading & Encoders");
    Serial.println("🧭 Priority: LEFT -> STRAIGHT -> RIGHT -> 180° BACK");
    Serial.println("========================================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    // 1. Check Button 1 (Start / Pause Toggle)
    if (millis() - lastBtnCheck > 250) {
        if (digitalRead(PIN_BTN1) == LOW) {
            solverActive = !solverActive;
            if (solverActive) {
                targetHeadingDeg = currentYawDeg;
                Serial.println("\n▶️ [LEFT-HAND SOLVER STARTED] Navigating maze...");
            } else {
                stopMotors();
                Serial.println("\n⏸️ [SOLVER PAUSED] Motors stopped.");
            }
            lastBtnCheck = millis();
        }

        // Button 2: Reset Gyro & Encoders
        if (digitalRead(PIN_BTN2) == LOW) {
            currentYawDeg = 0.0f;
            targetHeadingDeg = 0.0f;
            encLeftTicks = 0;
            encRightTicks = 0;
            Serial.println("\n🔄 [RESET] Heading and Encoders zeroed!");
            lastBtnCheck = millis();
        }
    }

    // 2. Execute Left-Hand Rule Maze Navigation if active
    if (solverActive) {
        executeLeftHandRuleStep();
    } else {
        stopMotors();
        readAllToFSensors();
        updateYaw();
    }

    // 3. Print Live Telemetry Stream every 200ms
    if (millis() - lastPrintTime >= 200) {
        lastPrintTime = millis();

        Serial.printf("[LSRB] L:%3dmm | FL:%3dmm | FRONT:%3dmm | FR:%3dmm | R:%3dmm || Gyro:%5.1f° || ",
                      tofDistMm[SENS_LEFT],
                      tofDistMm[SENS_FL],
                      tofDistMm[SENS_FRONT],
                      tofDistMm[SENS_FR],
                      tofDistMm[SENS_RIGHT],
                      currentYawDeg);

        if (solverActive) {
            Serial.println("🤖 SOLVER RUNNING");
        } else {
            Serial.println("⏸️ PAUSED (Press BTN1 to Start)");
        }
    }

    delay(4);
}
