#include "motors.h"

MotoronI2C mc;
// Motor default address: 16
// https://www.pololu.com/docs/0J84/3.5

Encoder encLeft(1,2);
Encoder encRight(3,4);

void motorSetup() {
  mc.reinitialize();           // Bytes: 0x96 0x74
  mc.disableCrc();             // Bytes: 0x8B 0x04 0x7B 0x43
  mc.clearResetFlag();         // Bytes: 0xA9 0x00 0x04
  mc.disableCommandTimeout();  // disables timeout to act the same as old motor drivers (TB6612FNG)

  attachInterrupt(2, updateLeftEncoder, CHANGE);  // Attach an interrupt to at least one of the encoder pins
  attachInterrupt(3, updateRightEncoder, CHANGE);  // Attach an interrupt to at least one of the encoder pins
}


// Xiao ESP32S3 Sense can have interrupt attached to each pin, but this needs to be done manually in code
// .read() needs to be called whenever a signal comes from the encoders to update its position
void IRAM_ATTR updateRightEncoder() {
  // Handle the interrupt
  encRight.read();
}
void IRAM_ATTR updateLeftEncoder() {
  // Handle the interrupt
  encLeft.read();
}

void setLeftPWM(int PWM) {
    mc.setSpeed(1, -PWM);
}

void setRightPWM(int PWM) {
    mc.setSpeed(2, -PWM);
}

