#include "pidRotate.h"

void turnTo(char direction) {
    double targetDirection = 0;
    double currentAngle = angle();
    double error = 0;
    double kp = 1.5;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double totalTime = micros();
    double previousTime = micros();
    double totalError = 0;
    double ki = 2;

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

   
    double startTime = micros();
    double sampleAngle = angle();

    while (1){

        if(leftMotorSpeed > 15|| abs(rightMotorSpeed) > 15) {
            startTime = micros();
        }

        if(micros() > startTime + 50e3){
            if(angle() == sampleAngle) {
                moveLeftMotor(0);
                moveRightMotor(0);
                return;
            }
            startTime = micros();
            sampleAngle = angle();
        }

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

        Serial.printf("Left Motor Speed: %lf\t Right Motor Speed: %lf\t Error: %lf\n", leftMotorSpeed, rightMotorSpeed, error);

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

    
        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);

    
    }

    moveLeftMotor(0);
    moveRightMotor(0);




}

    

