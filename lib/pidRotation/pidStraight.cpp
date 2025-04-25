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

        double encoderkP = 1.5, encoderkD = 1;
        double anglekP = 3, anglekD = 0; //28 before

        // distance *= 0.9;
        int numTicks = (90 * distance) / (DIA * PI); //Num ticks that we need to travel

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

        int currleft = getLeftEncoder();
        int currright = getRightEncoder();
        int newLeft = currleft;
        int newRight = currright;

        int currentLeftError = numTicks - currleft;
        int currentRightError = numTicks - currright;
        int previousLeftError = numTicks - currleft;
        int previousRightError = numTicks - currright;
    
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
            //Calculating left and right motor speed

            leftEncoderDeriv = (currentLeftError - previousLeftError) / (totalTime - previousTime);
            rightEncoderDeriv = (currentRightError - previousRightError) / (totalTime - previousTime);

            currentLeftError = constrain(currentLeftError, 0, maxPWM);
            currentRightError = constrain(currentRightError, 0, maxPWM);    

            // Serial.printf("old left speed: %lf\told right speed: %lf\n",oldLeftSpeed,oldRightSpeed);

            oldLeftSpeed = leftMotorSpeed;
            oldRightSpeed = rightMotorSpeed;

            if(currentLeftError < 50) {
                anglekP = currentLeftError/70;
            }

            leftMotorSpeed = (encoderkP * currentLeftError) + (anglekP * angleError + anglekD * derivative) + encoderkD * leftEncoderDeriv;
            rightMotorSpeed = (encoderkP * currentRightError) - (anglekP * angleError + anglekD * derivative) + encoderkD * rightEncoderDeriv;

            leftMotorSpeed = constrain(leftMotorSpeed, 0, maxPWM);
            rightMotorSpeed = constrain(rightMotorSpeed, 0, maxPWM);

            if(leftMotorSpeed > 20|| rightMotorSpeed > 20) {
                startTime = millis();
            }


            if(startTime + 50 < millis()) {
                moveLeftMotor(0);
                moveRightMotor(0);
                break;
            }

            oldLeftSpeed = leftMotorSpeed;
            oldRightSpeed = rightMotorSpeed;
            // if(derivative != 0)  Serial.printf("angle derivative: %lf\n", derivative);

            // if(rightEncoderDeriv != 0 || leftEncoderDeriv != 0) {
            //     Serial.printf("left derivative: %lf\t right deriviatve: %lf\n", leftEncoderDeriv, rightEncoderDeriv);
            // }
            
          Serial.printf("left speed: %lf\tright speed: %lf\n",leftMotorSpeed,rightMotorSpeed);
        
        
            // leftMotorSpeed = 125 + (kp * error) - (kd * derivative);
            // rightMotorSpeed = 125 - (kp * error) + (kd * derivative);
            // rightMotorSpeed *= 1.4;

            //Constraining the motor speeds
            // if (leftMotorSpeed < minPWM)
            // {
            //     leftMotorSpeed = minPWM;
            // }
            // else if (leftMotorSpeed > maxPWM)
            // {
            //     leftMotorSpeed = maxPWM;
            // }
            // if (rightMotorSpeed < minPWM)
            // {
            //     rightMotorSpeed = minPWM;
            // }
            // else if (rightMotorSpeed > maxPWM)
            // {
            //     rightMotorSpeed = maxPWM;
            // }
            
            moveLeftMotor(leftMotorSpeed);
            moveRightMotor(rightMotorSpeed);

            previousTime = totalTime;
            previousAngle = currentAngle;

            currleft = getLeftEncoder();
            currright = -1 * getRightEncoder();

            previousLeftError = currentLeftError;
            previousRightError = currentRightError;
            currentLeftError = numTicks - currleft;
            currentRightError = numTicks - currright;

            // Serial.printf("Left : %d\t Right : %d\n", currentLeftError,currentRightError);
        }

        moveLeftMotor(0);
        moveRightMotor(0);
        Serial.println("Done");
    }
