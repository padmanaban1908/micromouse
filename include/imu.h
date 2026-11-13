#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define MPU6050_ADDR 0x68

extern float gyroZOffset;
extern float currentYawDeg;
extern unsigned long lastGyroMicros;

inline void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(0x08); // +/- 500 deg/s (65.5 LSB/(deg/s))
    Wire.endTransmission(true);

    delay(100);

    Serial.print("Calibrating MPU6050 Gyro... Please keep bot stationary...");
    long sum = 0;
    for (int i = 0; i < 500; i++) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);
        if (Wire.available() >= 2) {
            int16_t rawZ = (Wire.read() << 8) | Wire.read();
            sum += rawZ;
        }
        delay(2);
    }
    gyroZOffset = (float)sum / 500.0f;
    Serial.printf(" Done! Bias: %.2f\n", gyroZOffset);
    lastGyroMicros = micros();
}

inline float readGyroZRate() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (uint8_t)true);
    if (Wire.available() >= 2) {
        int16_t rawZ = (Wire.read() << 8) | Wire.read();
        return (float)(rawZ - gyroZOffset) / 65.5f; // deg/s
    }
    return 0.0f;
}

inline void updateYaw() {
    unsigned long now = micros();
    float dt = (now - lastGyroMicros) / 1000000.0f;
    lastGyroMicros = now;
    if (dt > 0.05f) dt = 0.005f;
    float rate = readGyroZRate();
    currentYawDeg += rate * dt;
}
