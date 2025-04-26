#include "pidRotate.h"

void turnTo(char direction) {
    double targetDirection = 0;
    double currentAngle = angle();
    double error = 0;
    double kp = 1.25;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double totalTime = micros();
    double previousTime = micros();
    double totalError = 0;
    double ki = 0;

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

   
    int startTime = micros();

    while (1){
        currentAngle = angle();
        totalTime = micros();

        error = targetDirection - currentAngle;

        while (error > 180){
            error -= 360;
        }
        
        while (error < -180){
            error += 360;
        }

        totalError +=error/(totalTime - previousTime);

        leftMotorSpeed = ki*totalError + kp * error;
        rightMotorSpeed = -ki*totalError + kp * -error;

        previousTime = totalTime;

        Serial.printf("Left Motor Speed: %lf\t", leftMotorSpeed);
        Serial.printf("Right Motor Speed: %lf\n", rightMotorSpeed);

       // Serial.printf("Total error: %lf\n", totalError);

        if (leftMotorSpeed > 100){
            leftMotorSpeed = 100;
        } else if (leftMotorSpeed < -100){
            leftMotorSpeed = -100;
        }
        if (rightMotorSpeed > 100){
            rightMotorSpeed = 100;
        } else if (rightMotorSpeed < -100){
            rightMotorSpeed = -100;
        }

        if(leftMotorSpeed > 20|| rightMotorSpeed > 20) {
            startTime = millis();
        }
        
        if(startTime + 50 < millis()) {
            moveLeftMotor(0);
            moveRightMotor(0);
            break;
        }
    
        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed * 1.2);

    
    }

    moveLeftMotor(0);
    moveRightMotor(0);




}

    

