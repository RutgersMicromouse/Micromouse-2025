#include <Arduino.h>
#include "Motors.h"
#define PI 3.1415926535897932384626433832795


// Encoder encRight(2,3);
// Encoder encLeft(10,11);

//IN1 and IN2 are for Motor A
//IN3 and IN4 are for Motor B

//Call analogWrite on the pin that is high

void setupMotors(){
  pinMode(AIN1_LEFTMOT, OUTPUT);
  pinMode(AIN2_LEFTMOT, OUTPUT);
  pinMode(BIN1_RIGHTMOT, OUTPUT);
  pinMode(BIN2_RIGHTMOT, OUTPUT);
}

void moveLeftMotor(uint8_t leftPWM, char direction) {
  if(direction == 'N') { //moving forward relative to front of robot
    digitalWrite(AIN1_LEFTMOT, HIGH);
    digitalWrite(AIN2_LEFTMOT, LOW);
    analogWrite(AIN1_LEFTMOT, leftPWM);
  } else {
    digitalWrite(AIN1_LEFTMOT, LOW);
    digitalWrite(AIN2_LEFTMOT, HIGH);
    analogWrite(AIN2_LEFTMOT, leftPWM);
  }
}

void moveRightMotor(uint8_t rightPWM, char direction) {
  if(direction == 'N') { //moving forward relative to front of robot
    digitalWrite(BIN1_RIGHTMOT, LOW);
    digitalWrite(BIN2_RIGHTMOT, HIGH);
    analogWrite(BIN2_RIGHTMOT, rightPWM);
  } else {
    digitalWrite(BIN1_RIGHTMOT, HIGH);
    digitalWrite(BIN2_RIGHTMOT, LOW);
    analogWrite(BIN1_RIGHTMOT, rightPWM);
  }

}

void stopMotors() {
  digitalWrite(AIN1_LEFTMOT, LOW);
  digitalWrite(AIN2_LEFTMOT, LOW);
  digitalWrite(BIN1_RIGHTMOT, LOW);
  digitalWrite(BIN2_RIGHTMOT, LOW);

  
}






