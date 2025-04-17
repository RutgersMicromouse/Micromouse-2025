#include "pidRotate.h"

double p = 1.1;
double d = 0;
double i = 8;

void turnTo(char direction) {
    double targetDirection = 0;
    double currentAngle = angle();
    double error = 0;
    double kp = 1;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double totalTime = micros();
    double previousTime = micros();
    double totalError = 0;
    double ki = 1;

    //Based on direction we want to go to, we set the target angle
    switch(direction) {
        case 'N':
            targetDirection = 0;
            break;
        case 'S':
            targetDirection = 180;
            break;
        case 'E':
            targetDirection = 90;
            break;
        case 'W':
            targetDirection = 270;
            break;
    }

    while (1){
        currentAngle = angle();
        totalTime = micros();

        error = targetDirection - currentAngle;

        if (error > 180){
            error -= 360;
        }
        
        if (error <= -180){
            error += 360;
        }

        totalError +=error/(totalTime - previousTime);

        leftMotorSpeed = ki*totalError + kp * error;
        rightMotorSpeed = -ki*totalError + kp * -error;

        previousTime = totalTime;

        Serial.printf("Left Motor Speed: %lf\t", leftMotorSpeed);
        Serial.printf("Right Motor Speed: %lf\n", rightMotorSpeed);

       // Serial.printf("Total error: %lf\n", totalError);

        if (leftMotorSpeed > 127){
            leftMotorSpeed = 127;
        } else if (leftMotorSpeed < -127){
            leftMotorSpeed = -127;
        }
        if (rightMotorSpeed > 127){
            rightMotorSpeed = 127;
        } else if (rightMotorSpeed < -127){
            rightMotorSpeed = -127;
        }

    

        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);

    
    }




}

    

