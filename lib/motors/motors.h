#pragma once

#include <Motoron.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>

// call encLeft.read() and encRight.read() to get encoder values
extern Encoder encLeft;
extern Encoder encRight;

void motorSetup();

void IRAM_ATTR updateRightEncoder();
void IRAM_ATTR updateLeftEncoder();

void setLeftPWM(int PWM);
void setRightPWM(int PWM);
