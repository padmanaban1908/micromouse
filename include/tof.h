#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

extern Adafruit_VL53L0X tofSensors[NUM_TOF];
extern uint16_t tofDistMm[NUM_TOF];

inline void initToFSensors() {
    // 1. Hold all sensors in hardware reset
    for (int i = 0; i < NUM_TOF; i++) {
        pinMode(TOF_XSHUT_PINS[i], OUTPUT);
        digitalWrite(TOF_XSHUT_PINS[i], LOW);
    }
    delay(20);

    // 2. Sequential power up and unique address reassignment
    for (int i = 0; i < NUM_TOF; i++) {
        digitalWrite(TOF_XSHUT_PINS[i], HIGH);
        delay(15);

        if (!tofSensors[i].begin(TOF_I2C_ADDRS[i])) {
            Serial.printf("⚠️ ToF Sensor %d (XSHUT %d) failed at 0x%02X!\n",
                          i, TOF_XSHUT_PINS[i], TOF_I2C_ADDRS[i]);
        } else {
            Serial.printf("✅ ToF Sensor %d online at 0x%02X\n",
                          i, TOF_I2C_ADDRS[i]);
        }
        delay(10);
    }
}

inline void readAllToFSensors() {
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
