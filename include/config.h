#pragma once
#include <Arduino.h>

// ============================================================================
// 1. PIN CONFIGURATION
// ============================================================================

// Left Motor (TB6612FNG)
#define PIN_PWMA 27
#define PIN_AIN1 14
#define PIN_AIN2 26

// Right Motor (TB6612FNG)
#define PIN_PWMB 32
#define PIN_BIN1 25
#define PIN_BIN2 33

// Quadrature Encoders (GPI Pins with external 4.7k pull-up resistors to 3.3V)
#define PIN_ENC_L_A 36 // VP
#define PIN_ENC_L_B 39 // VN
#define PIN_ENC_R_A 34
#define PIN_ENC_R_B 35

// I2C Bus
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// 5x Time-of-Flight (VL53L0X) XSHUT Pins
#define NUM_TOF 5
const int TOF_XSHUT_PINS[NUM_TOF] = {23, 19, 18, 17, 16};
const uint8_t TOF_I2C_ADDRS[NUM_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34};

// ToF Indices
#define SENS_LEFT   0  // Left 90° (Pin 23)
#define SENS_FL     1  // Front-Left 45° (Pin 19)
#define SENS_FRONT  2  // Front Center 0° (Pin 18)
#define SENS_FR     3  // Front-Right 45° (Pin 17)
#define SENS_RIGHT  4  // Right 90° (Pin 16)

// Pushbuttons (Active LOW)
#define PIN_BTN1 4   // Button 1: Start Search Run
#define PIN_BTN2 13  // Button 2: Start High-Speed Smooth Turn Run

// ============================================================================
// 2. CONSTANTS & TUNING PARAMETERS
// ============================================================================
#define PWM_FREQ 20000
#define PWM_RES  8
#define CH_MOT_L 0
#define CH_MOT_R 1

// Odometry Calibration
#define TICKS_PER_MM 3.58f
#define CELL_TICKS   (180.0f * TICKS_PER_MM) // ~644 ticks per cell

// ToF Wall Detection Thresholds (mm)
#define WALL_SIDE_THRESHOLD_MM  130
#define WALL_FRONT_THRESHOLD_MM 150
#define NOMINAL_SIDE_DIST_MM    84

// Speeds (0-255 PWM)
#define SPEED_SEARCH_CRUISE 150
#define SPEED_FAST_CRUISE   220
#define SPEED_SMOOTH_TURN   160

// PID Gains
#define KP_WALL 0.50f
#define KD_WALL 0.25f

#define KP_GYRO 3.20f
#define KD_GYRO 0.80f

// Maze Definitions
#define MAZE_SIZE 16

#define WALL_NORTH 0x01
#define WALL_EAST  0x02
#define WALL_SOUTH 0x04
#define WALL_WEST  0x08
#define VISITED    0x10

enum Direction {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
};
