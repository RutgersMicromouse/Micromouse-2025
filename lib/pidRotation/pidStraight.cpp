#include "pidStraight.h"
#define DIA 32
#define PI 3.1415926535897932384626433832795

double minPWM = 100;
double maxPWM = 150;

void straight(char direction, int distance)
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
        double kd = 100;

        int numTicks = (90 * distance) / (DIA * PI); //Num ticks that we need to travel
        distance = distance * 0.9;

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

        //Clear the encoder count
        encLeft.clearCount();
        encRight.clearCount();

        int currleft = abs(getLeftEncoder());
        int currright = abs(getRightEncoder());

        int currentLeftError = abs(numTicks - currleft);
        int currentRightError = abs(numTicks - currright);
        /* 5*/
        while (currentLeftError > 5)
        {
            //Angle readings for IMU
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
            
            //Calculating derivative
            derivative = (currentAngle - previousAngle) / (totalTime - previousTime);
            if(derivative != 0){
                Serial.printf("derivative %lf\n",derivative);
            }
            //Calculating left and right motor speed
            leftMotorSpeed = 125 + (kp * error) - (kd * derivative);
            rightMotorSpeed = 125 - (kp * error) + (kd * derivative);
            rightMotorSpeed *= 1.4;

            //Constraining the motor speeds
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

            // Serial.printf("error: %lf\tleft speed: %lf\tright speed: %lf\n",error,leftMotorSpeed,rightMotorSpeed);
            
            moveLeftMotor(leftMotorSpeed);
            moveRightMotor(rightMotorSpeed);
            previousTime = totalTime;
            previousAngle = currentAngle;

            currleft = abs(getLeftEncoder());
            currright = abs(getRightEncoder());

            currentLeftError = abs(numTicks - currleft);
            currentRightError = abs(numTicks - currright);

            // Serial.printf("Left : %d\t Right : %d\n", currleft,currright);
        }

        moveLeftMotor(0);
        moveRightMotor(0);
        delay(1000);
    }
