#pragma once
#include <Arduino.h>
#include <esp_arduino_version.h>
#include "config.h"

// Dual compatibility helper for ESP32 Arduino Core 2.x and 3.x
inline void pwmWriteLeft(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(PIN_PWMA, val);
#else
    ledcWrite(CH_MOT_L, val);
#endif
}

inline void pwmWriteRight(uint32_t val) {
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    ledcWrite(PIN_PWMB, val);
#else
    ledcWrite(CH_MOT_R, val);
#endif
}

inline void initMotors() {
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    // ESP32 Arduino Core 3.x API
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
#else
    // ESP32 Arduino Core 2.x API
    ledcSetup(CH_MOT_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMA, CH_MOT_L);
    ledcSetup(CH_MOT_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMB, CH_MOT_R);
#endif
}

inline void setMotorLeft(int speed) {
    speed = constrain(speed, -255, 255);
    // Verified on robot: Forward is AIN1 = LOW, AIN2 = HIGH
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

inline void setMotorRight(int speed) {
    speed = constrain(speed, -255, 255);
    // Verified on robot: Forward is BIN1 = HIGH, BIN2 = LOW
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

inline void setMotorRaw(int leftPwm, int rightPwm) {
    setMotorLeft(leftPwm);
    setMotorRight(rightPwm);
}

inline void stopMotors() {
    setMotorRaw(0, 0);
}
