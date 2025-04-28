#include "pidStraight.h"
#define DIA 32
#define PI 3.1415926535897932384626433832795

double minPWM = 75;
double maxPWM = 175;

void straight(char direction, int distance)
{
        double currentAngle = angle();
        double targetDirection = 0;
        double leftMotorSpeed = 0;
        double rightMotorSpeed = 0;
       // double kp = 8;
        double previousTime = millis();
        double totalTime = millis();
        double previousAngle = 0;
        double derivative = angle();
      //  double kd = 100;

        double encoderkP, encoderkD;
        double anglekP, anglekD;

        if(distance == 160) {
            encoderkP = 0.9;
            encoderkD = 0.1;
            anglekP = 1.5;
            anglekD = 2.1; 
        } else if(distance > 160){
            encoderkP = 1.5;
            encoderkD = 5;
            anglekP = 0;  
            anglekD = 0;
        } else {
            encoderkP = 1.3;
            encoderkD = 0.5;
            anglekP = 0;
            anglekD = 0;
        }

        int numTicks = (98 * distance) / (DIA * PI); //Num ticks that we need to travel

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

        int currleft = getLeftEncoder();
        int currright = getRightEncoder();

        int newLeft = currleft;
        int newRight = currright;

        int currentLeftError = numTicks - currleft;
        int currentRightError = numTicks - currright;
        int previousLeftError = numTicks - currleft;
        int previousRightError = numTicks - currright;

        int currentAverageError, previousAverageError;
    
        double angleError;
        double leftEncoderDeriv = 0;
        double rightEncoderDeriv = 0;

        double oldLeftSpeed = 0;
        double oldRightSpeed = 0;

        int startTime = millis();
        while (1)
        {
            //Angle readings for IMU
            currentAngle = angle();
            angleError = targetDirection - currentAngle;
            totalTime = millis();
            while(angleError > 180)
            {
                angleError -= 360;
            }
            while(angleError <= -180)
            {
                angleError += 360;
            }
        
            //Calculating derivative
            derivative = (currentAngle - previousAngle) / (totalTime - previousTime);
            leftEncoderDeriv = (currentLeftError - previousLeftError) / (totalTime - previousTime);
            rightEncoderDeriv = (currentRightError - previousRightError) / (totalTime - previousTime);

            //Calculating left and right motor speed
            leftMotorSpeed = (encoderkP * currentLeftError) + (anglekP * angleError + anglekD * derivative) + encoderkD * leftEncoderDeriv;
            rightMotorSpeed = (encoderkP * currentRightError) - (anglekP * angleError + anglekD * derivative) + encoderkD * rightEncoderDeriv;
            leftMotorSpeed = constrain(leftMotorSpeed, 0, maxPWM);
            rightMotorSpeed = constrain(rightMotorSpeed, 0, maxPWM);

            leftMotorSpeed *= 1.03; //Slight adjusment of left motorspeed


            if(leftMotorSpeed > 20|| rightMotorSpeed > 20) {
                startTime = millis();
            }

            if(startTime + 100 < millis()) {
                moveLeftMotor(0);
                moveRightMotor(0);
                break;
            }

            moveLeftMotor(leftMotorSpeed);
            moveRightMotor(rightMotorSpeed);

            previousTime = totalTime;
            previousAngle = currentAngle;

            previousAverageError = currentAverageError;
        
            currleft = getLeftEncoder();
            currright = -1 * getRightEncoder();

            previousLeftError = currentLeftError;
            previousRightError = currentRightError;
            currentLeftError = numTicks - currleft;
            currentRightError = numTicks - currright;

            currentAverageError = (currentLeftError + currentRightError)/2;

            Serial.printf("left motor speed: %lf\tright motor speed: %lf\n", leftMotorSpeed, rightMotorSpeed);

        }

        moveLeftMotor(0);
        moveRightMotor(0);
        Serial.println("Done");
    }


