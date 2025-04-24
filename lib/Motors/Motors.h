 #ifndef motors_h
#define motors_h

#include <Encoder.h>

#define AIN1_LEFTMOT 10 //IN1
#define AIN2_LEFTMOT 11 //IN2
#define BIN1_RIGHTMOT 13 //IN3 
#define BIN2_RIGHTMOT 12 //IN4

#define encRIGHTPIN1 6                                                                             
#define encRIGHTPIN2 7

#define encLEFTPIN1 8
#define encLEFTPIN2 9

extern Encoder encLeft;
extern Encoder encRight;


void moveLeftMotor(int PWM);
void moveRightMotor(int PWM);
void stopMotors();
void setupMotors();
void setupEncoders();
int32_t getLeftEncoder();

#endif