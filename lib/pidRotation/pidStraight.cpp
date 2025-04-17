#include "pidStraight.h"

double minPWM = 100;
double maxPWM = 200;

void straight(char direction){
    double currentAngle = angle();
    double targetDirection = 0;
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;
    double kp = 1.8;
    double previousTime = millis();
    double totalTime = millis();
    double previousAngle = 0;
    double derivative = angle();
    double kd = 15; //Previous value was 2

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

    while(1){
        currentAngle = angle();
        double error = targetDirection - currentAngle;

        totalTime = millis();

        if (error > 180){
            error -= 360;
        }
        
        if (error <= -180){
            error += 360;
        }


        derivative = (currentAngle - previousAngle)/(totalTime - previousTime);

        //Serial.printf("Error: %lf\n", error);

        // if (derivative != 0){
        //     Serial.printf("Derivative: %lf\n", derivative);
        // }

        leftMotorSpeed = 150 + kp*error - kd*derivative;
        rightMotorSpeed = 150 - kp*error + kd*derivative;
        rightMotorSpeed *= 1.15;

        if (leftMotorSpeed < minPWM){
            leftMotorSpeed = minPWM;
        }
        else if (leftMotorSpeed > maxPWM){
            leftMotorSpeed = maxPWM;
        }

        if (rightMotorSpeed < minPWM){
            rightMotorSpeed = minPWM;
        }
        else if (rightMotorSpeed > maxPWM){
            rightMotorSpeed = maxPWM;
        }
        
        Serial.printf("%lf\t%lf\t",error,leftMotorSpeed);
        Serial.printf("%lf\n",rightMotorSpeed);
        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);

        previousTime = totalTime;
        previousAngle = currentAngle;

        // Serial.printf("Left Motor Speed: %lf\t", leftMotorSpeed);
        // Serial.printf("Right Motor Speed: %lf\n", rightMotorSpeed);

    }

}
