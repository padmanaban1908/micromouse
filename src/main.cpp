/**
 * ============================================================================
 * 🐭 AUTONOMOUS MICROMOUSE ESP32 MAIN FIRMWARE (PlatformIO)
 * ============================================================================
 * Hardware Pin Specifications:
 *   - Left Motor   : PWMA=27, AIN1=14, AIN2=26
 *   - Right Motor  : PWMB=32, BIN1=25, BIN2=33 (Inverted for forward symmetry)
 *   - Encoders     : Left=(36, 39), Right=(34, 35) + 4.7k external pull-ups
 *   - I2C Bus      : SDA=21, SCL=22 (400 kHz Fast I2C)
 *   - 5x ToF XSHUT : 23, 19, 18, 17, 16 (VL53L0X sensors)
 *   - IMU          : MPU-6050 (Direct I2C register access @ 0x68)
 *   - Pushbuttons  : BTN1=4 (Search Run), BTN2=13 (High-Speed Smooth Turn Run)
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motors.h"
#include "encoders.h"
#include "imu.h"
#include "tof.h"
#include "floodfill.h"
#include "motion.h"

// ============================================================================
// GLOBAL VARIABLE DEFINITIONS
// ============================================================================
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

float gyroZOffset = 0.0f;
float currentYawDeg = 0.0f;
unsigned long lastGyroMicros = 0;

Adafruit_VL53L0X tofSensors[NUM_TOF];
uint16_t tofDistMm[NUM_TOF] = {999, 999, 999, 999, 999};

uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];
uint8_t cellDist[MAZE_SIZE][MAZE_SIZE];

int robotX = 0;
int robotY = 0;
Direction robotDir = DIR_NORTH;

CellCoord floodQueue[256];
int qHead = 0;
int qTail = 0;

// Encoder Interrupt Handlers with Inversion Flag Support
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
    if (digitalRead(PIN_ENC_R_B)) encRightTicks--; else encRightTicks++;
#endif
}

// Operating State Machine
enum MouseState {
    STATE_WAIT_USER,
    STATE_SEARCH_RUN,
    STATE_RETURN_TO_START,
    STATE_SPEED_RUN,
    STATE_DONE
};

MouseState mouseState = STATE_WAIT_USER;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================================");
    Serial.println("🐭 ESP32 MICROMOUSE INITIALIZING (50% Speed Profile)...");
    Serial.println("========================================================");

    // 1. Initialize Buttons
    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);

    // 2. Initialize Motors & Encoders
    initMotors();
    initEncoders();

    // 3. Initialize I2C Bus on SDA=21, SCL=22
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    // 4. Initialize MPU-6050
    initMPU6050();

    // 5. Initialize 5x VL53L0X ToF Sensors
    initToFSensors();

    // 6. Initialize 16x16 Maze Field
    initMaze();
    floodfill(true); // Pre-calculate distances to center goal

    Serial.println("========================================================");
    Serial.println("READY FOR ACTION (50% SPEED):");
    Serial.println("  👉 Press BTN1 (Pin 4)  : Start Search Run (Exploration)");
    Serial.println("  👉 Press BTN2 (Pin 13) : Start Speed Run (SMOOTH 90° TURNS)");
    Serial.println("========================================================");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    switch (mouseState) {
        case STATE_WAIT_USER: {
            if (digitalRead(PIN_BTN1) == LOW) {
                delay(300); // Debounce
                Serial.println("🚀 [RUN 1] Starting Autonomous Search Run (50% Speed)...");
                mouseState = STATE_SEARCH_RUN;
            } else if (digitalRead(PIN_BTN2) == LOW) {
                delay(300); // Debounce
                Serial.println("⚡ [RUN 2] Starting High-Speed Run with Smooth 90° Turns (50% Speed)!");
                mouseState = STATE_SPEED_RUN;
            }
            delay(10);
            break;
        }

        case STATE_SEARCH_RUN: {
            bool wallsChanged = updateWallsFromSensors();
            if (wallsChanged) {
                floodfill(true);
            }

            if (isAtGoal()) {
                stopMotors();
                Serial.printf("🎉 Goal Reached at (%d, %d)! Re-targeting Start...\n", robotX, robotY);
                delay(1500);
                floodfill(false); // Destination is (0, 0)
                mouseState = STATE_RETURN_TO_START;
                break;
            }

            Direction nextDir = getBestNextDirection();
            navigateToNextCell(nextDir, false, SPEED_SEARCH_CRUISE);
            break;
        }

        case STATE_RETURN_TO_START: {
            bool wallsChanged = updateWallsFromSensors();
            if (wallsChanged) {
                floodfill(false);
            }

            if (isAtStart()) {
                stopMotors();
                Serial.println("🏁 Safely Returned to Start! Optimal Map Computed.");
                delay(2000);
                floodfill(true); // Prepare optimal path to goal
                mouseState = STATE_WAIT_USER; // Ready for user to press BTN2 for Speed Run!
                break;
            }

            Direction nextDir = getBestNextDirection();
            navigateToNextCell(nextDir, false, SPEED_SEARCH_CRUISE);
            break;
        }

        case STATE_SPEED_RUN: {
            if (isAtGoal()) {
                stopMotors();
                Serial.println("🏆 Speed Run Finished with Maximum Velocity!");
                mouseState = STATE_DONE;
                break;
            }

            Direction nextDir = getBestNextDirection();
            // Engage Smooth Continuous 90° Turns at controlled speed!
            navigateToNextCell(nextDir, true, SPEED_FAST_CRUISE);
            break;
        }

        case STATE_DONE:
        default:
            stopMotors();
            delay(100);
            break;
    }
}
