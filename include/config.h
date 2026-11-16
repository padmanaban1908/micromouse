#pragma once
#include <Arduino.h>

// ============================================================================
// 1. PIN CONFIGURATION (VERIFIED ON ROBOT)
// ============================================================================

// Motor Driver (TB6612FNG)
#define PIN_PWMA 27
#define PIN_AIN1 14
#define PIN_AIN2 26

#define PIN_PWMB 32
#define PIN_BIN1 25
#define PIN_BIN2 33

// Motor Forward Directions (Verified on Robot):
// Left:  AIN1=LOW,  AIN2=HIGH
// Right: BIN1=HIGH, BIN2=LOW

// Quadrature Encoders (GPIO 34-39 input-only, with external 4.7k pull-ups to 3.3V)
// Verified on robot: left encoder on 34/35, right on 36/39
#define PIN_ENC_L_A 34
#define PIN_ENC_L_B 35
#define PIN_ENC_R_A 36
#define PIN_ENC_R_B 39

#define L_ENC_DIR  1
#define R_ENC_DIR -1

// Encoders CPR & Wheel Geometry (4x Quadrature Decoding)
#define CPR_LEFT       2788.0f
#define CPR_RIGHT      2788.0f
#define WHEEL_DIAM_MM  33.5f
#define MM_PER_COUNT   (3.14159265f * WHEEL_DIAM_MM / CPR_LEFT) // ~0.03775 mm/count

// I2C Bus
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// 5x Time-of-Flight (VL53L0X) XSHUT Pins
#define NUM_TOF 5
const int TOF_XSHUT_PINS[NUM_TOF]   = {23, 19, 18, 17, 16};
const uint8_t TOF_I2C_ADDRS[NUM_TOF] = {0x30, 0x31, 0x32, 0x33, 0x34};

#define SENS_LEFT   0  // Left 90° (Pin 23)
#define SENS_FL     1  // Front-Left 45° (Pin 19)
#define SENS_FRONT  2  // Front Center 0° (Pin 18)
#define SENS_FR     3  // Front-Right 45° (Pin 17)
#define SENS_RIGHT  4  // Right 90° (Pin 16)

// Pushbuttons
#define PIN_BTN1 13  // Button 1: Start / Pause (or Verify)
#define PIN_BTN2 4   // Button 2: Zero Counters / Reset

// PWM Parameters
#define PWM_FREQ 5000
#define PWM_RES  8
#define CH_MOT_L 0
#define CH_MOT_R 1

// Speeds (0-255 PWM)
#define SPEED_SEARCH_CRUISE 75
#define SPEED_FAST_CRUISE   110
#define SPEED_SMOOTH_TURN   80

// Sensor Thresholds
#define SIDE_OPEN_THRESHOLD_MM  150
#define FRONT_WALL_THRESHOLD_MM 110
#define NOMINAL_SIDE_DIST_MM    84

#define KP_WALL 0.35f
#define KP_GYRO 2.50f
#define KD_GYRO 0.60f
