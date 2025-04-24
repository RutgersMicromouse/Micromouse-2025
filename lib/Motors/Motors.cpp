#include <Arduino.h>
#include "Motors.h"
#define PI 3.1415926535897932384626433832795



ESP32Encoder encLeft;
ESP32Encoder encRight;

//IN1 and IN2 are for Motor A
//IN3 and IN4 are for Motor B

//Call analogWrite on the pin that is high
int32_t getLeftEncoder(){
  return encLeft.getCount();
}

int32_t getRightEncoder(){
  return encRight.getCount();
}
void setupEncoders() {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encRight.attachHalfQuad(encRIGHTPIN1,encRIGHTPIN2); 
    encLeft.attachHalfQuad(encLEFTPIN1,encLEFTPIN2);
    encLeft.clearCount();
    encRight.clearCount();
}

void setupMotors(){
  pinMode(AIN1_LEFTMOT, OUTPUT);
  pinMode(AIN2_LEFTMOT, OUTPUT);
  pinMode(BIN1_RIGHTMOT, OUTPUT);
  pinMode(BIN2_RIGHTMOT, OUTPUT);

  setupEncoders();

  Serial.println("Motors are setup!");
}



static void actualMoveLeftMotor(uint8_t leftPWN, char direction){
  if(direction == 'N') { //moving forward relative to front of robot
    digitalWrite(AIN1_LEFTMOT, HIGH);
    digitalWrite(AIN2_LEFTMOT, LOW);
    analogWrite(AIN1_LEFTMOT, leftPWN);
  } else {
    digitalWrite(AIN1_LEFTMOT, LOW);
    digitalWrite(AIN2_LEFTMOT, HIGH);
    analogWrite(AIN2_LEFTMOT, leftPWN);
  }
}
void moveLeftMotor(int PWM) {
  if(PWM >= 0){
    actualMoveLeftMotor(PWM,'N');
  }
  else{
    actualMoveLeftMotor(-PWM,'X');
  }
}

static void actualMoveRightMotor(uint8_t rightPWM, char direction) {
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

void moveRightMotor(int PWM) {
  if(PWM >= 0){
    actualMoveRightMotor(PWM,'N');
  }
  else{
    actualMoveRightMotor(-PWM,'X');
  }
}

void stopMotors() {
  digitalWrite(AIN1_LEFTMOT, LOW);
  digitalWrite(AIN2_LEFTMOT, LOW);
  digitalWrite(BIN1_RIGHTMOT, LOW);
  digitalWrite(BIN2_RIGHTMOT, LOW);

}