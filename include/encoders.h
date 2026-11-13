#pragma once
#include <Arduino.h>
#include "config.h"

extern volatile long encLeftTicks;
extern volatile long encRightTicks;

void IRAM_ATTR isrEncoderLeft();
void IRAM_ATTR isrEncoderRight();

inline void initEncoders() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, RISING);
}
