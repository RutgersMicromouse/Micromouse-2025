#include "motors.h"
#define PI 3.1415926535897932384626433832795

ESP32Encoder encLeft;
ESP32Encoder encRight;

int32_t getLeftEncoder(){
  return -1 * encLeft.getCount();
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

  pinMode(BIN1, OUTPUT);  
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(AIN1, OUTPUT);  
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  digitalWrite(STBY, HIGH);

  setupEncoders();

  Serial.println("Motors are setup!");
}

void moveLeftMotor(int PWM) {
    if(PWM > 0) {
        digitalWrite(BIN1, HIGH);   
        digitalWrite(BIN2, LOW);    
        analogWrite(PWMB, PWM);
    } else {
        digitalWrite(BIN1, LOW);    
        digitalWrite(BIN2, HIGH);   
        analogWrite(PWMB, -PWM);
    }
}


void moveRightMotor(int PWM) {
    if(PWM > 0) {
        digitalWrite(AIN1, LOW);    
        digitalWrite(AIN2, HIGH);   
        analogWrite(PWMA, PWM);
    } else {
        digitalWrite(AIN1, HIGH);   
        digitalWrite(AIN2, LOW);    
        analogWrite(PWMA, -PWM);
    }
}


void stopMotors() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}


