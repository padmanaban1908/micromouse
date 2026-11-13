#pragma once
#include <Arduino.h>
#include "config.h"

inline void initMotors() {
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
#else
    ledcSetup(CH_MOT_L, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMA, CH_MOT_L);
    ledcSetup(CH_MOT_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMB, CH_MOT_R);
#endif
}

inline void setMotorRaw(int leftPwm, int rightPwm) {
    leftPwm  = constrain(leftPwm, -255, 255);
    rightPwm = constrain(rightPwm, -255, 255);

    // Left Motor
    if (leftPwm >= 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMA, leftPwm);
#else
        ledcWrite(CH_MOT_L, leftPwm);
#endif
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMA, -leftPwm);
#else
        ledcWrite(CH_MOT_L, -leftPwm);
#endif
    }

    // Right Motor
    if (rightPwm >= 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMB, rightPwm);
#else
        ledcWrite(CH_MOT_R, rightPwm);
#endif
    } else {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_PWMB, -rightPwm);
#else
        ledcWrite(CH_MOT_R, -rightPwm);
#endif
    }
}

inline void stopMotors() {
    setMotorRaw(0, 0);
}
