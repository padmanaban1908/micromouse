#pragma once
#include <Arduino.h>
#include "config.h"
#include "soc/gpio_struct.h"

extern volatile int32_t encLeftTicks;
extern volatile int32_t encRightTicks;
extern volatile uint8_t leftEncState;
extern volatile uint8_t rightEncState;

// Quadrature lookup: index = (prevAB << 2) | newAB
static const int8_t QDEC_TABLE[16] = {
   0, -1,  1,  2,
   1,  0,  2, -1,
  -1,  2,  0,  1,
   2,  1, -1,  0
};

static inline uint8_t IRAM_ATTR readEncAB(uint8_t pinA, uint8_t pinB) {
  uint32_t in = GPIO.in1.val;
  return (((in >> (pinA - 32)) & 1) << 1) | ((in >> (pinB - 32)) & 1);
}

void IRAM_ATTR isrEncoderLeft();
void IRAM_ATTR isrEncoderRight();

inline void initEncoders() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    leftEncState  = readEncAB(PIN_ENC_L_A, PIN_ENC_L_B);
    rightEncState = readEncAB(PIN_ENC_R_A, PIN_ENC_R_B);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncoderLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncoderLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncoderRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncoderRight, CHANGE);
}
