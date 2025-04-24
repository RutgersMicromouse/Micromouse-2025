#include "pidStraight.h"
#define DIA 32
#define PI 3.1415926535897932384626433832795

double minPWM = 100;
double maxPWM = 150;

void straight(char direction)
{
        double currentAngle = angle();
        double targetDirection = 0;
        double leftMotorSpeed = 0;
        double rightMotorSpeed = 0;
        double kp = 8;
        double previousTime = millis();
        double totalTime = millis();
        double previousAngle = 0;
        double derivative = angle();
        double kd = 0;

        double distance = 10;
        int numTicks = (360 * distance) / (DIA * PI);

        switch (direction)
        {
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
        encLeft.clearCount();
        encRight.clearCount();
        int currleft = abs(getLeftEncoder());
        int currright = abs(getRightEncoder());
        while (1)
        {
            currentAngle = angle();
            double error = targetDirection - currentAngle;

            totalTime = millis();

            while(error > 180)
            {
                error -= 360;
            }

            while(error <= -180)
            {
                error += 360;
            }

            derivative = (currentAngle - previousAngle) / (totalTime - previousTime);

            leftMotorSpeed = 125 + (kp * error) + (kd * derivative);
            rightMotorSpeed = 125 - (kp * error) - (kd * derivative);
            rightMotorSpeed *= 1.4;

            if (leftMotorSpeed < minPWM)
            {
                leftMotorSpeed = minPWM;
            }
            else if (leftMotorSpeed > maxPWM)
            {
                leftMotorSpeed = maxPWM;
            }

            if (rightMotorSpeed < minPWM)
            {
                rightMotorSpeed = minPWM;
            }
            else if (rightMotorSpeed > maxPWM)
            {
                rightMotorSpeed = maxPWM;
            }
            Serial.printf("error: %lf\tleft speed: %lf\tright speed: %lf\n",error,leftMotorSpeed,rightMotorSpeed);
            moveLeftMotor(leftMotorSpeed);
            moveRightMotor(rightMotorSpeed);
            previousTime = totalTime;
            previousAngle = currentAngle;
            currleft = abs(getLeftEncoder());
            currright = abs(getRightEncoder());
        }

        moveLeftMotor(0);
        moveRightMotor(0);
        delay(1000);
    }
