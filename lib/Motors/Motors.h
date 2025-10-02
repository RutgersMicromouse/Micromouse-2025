 #ifndef motors_h
#define motors_h

#include <ESP32Encoder.h>
#include <Arduino.h>

#define AIN1 14 
#define AIN2 13 
#define BIN1 16
#define BIN2 17 

#define PWMA 12
#define PWMB 18
#define STBY 15

#define encRIGHTPIN1 8                                                                       
#define encRIGHTPIN2 7

#define encLEFTPIN1 10
#define encLEFTPIN2 9

extern ESP32Encoder encLeft;
extern ESP32Encoder encRight;


void moveLeftMotor(int PWM);
void moveRightMotor(int PWM);

void stopMotors();
void setupMotors();
void setupEncoders();

int32_t getLeftEncoder();
int32_t getRightEncoder();

#endif