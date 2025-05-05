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
        double previousTime = millis();
        double totalTime = millis();
        double previousAngle = 0;
        double derivative = angle();

        double encoderkP, encoderkD;
        double anglekP, anglekD;

        if(distance == 160) {
            encoderkP = 0.9;
            encoderkD = 0.1;
            anglekP = 2;
            anglekD = 2.3; 
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

        double tnumTicks = (102 * distance) / (DIA * PI); //Num ticks that we need to travel
        tnumTicks *= 1.00;
        int numTicks = tnumTicks;
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

        int previousLeftPosition = currleft;
        int previousRightPosition = currright;

        int currentLeftError = numTicks - currleft;
        int currentRightError = numTicks - currright;
        int previousLeftError = numTicks - currleft;
        int previousRightError = numTicks - currright;

        int currentAverageError, previousAverageError;
    
        double angleError;
        double leftEncoderDeriv = 0;
        double rightEncoderDeriv = 0;

        // Variables for stall detection
        int stallCheckInterval = 250; // Check for stalls every 250ms
        int lastStallCheck = millis();
        int stallThreshold = 2; // Minimum encoder ticks expected in each interval
        bool isStalled = false;
        int stallCount = 0;
        int maxStallCount = 3; // Number of consecutive stall detections before aborting

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

            rightMotorSpeed *= 1.1; //Slight adjusment of left motorspeed

            // Check for stall condition at regular intervals
            if (millis() - lastStallCheck >= stallCheckInterval) {
                currleft = getLeftEncoder();
                currright = -1 * getRightEncoder();
                
                // Check if the encoder values haven't changed significantly despite power being applied
                int leftChange = abs(currleft - previousLeftPosition);
                int rightChange = abs(currright - previousRightPosition);
                
                // If motors are running but encoders show little movement, consider it stalled
                if ((leftMotorSpeed > 30 || rightMotorSpeed > 30) && 
                    (leftChange < stallThreshold && rightChange < stallThreshold)) {
                    stallCount++;
                    isStalled = true;
                    Serial.printf("Potential stall detected (%d/%d): Left change: %d, Right change: %d\n", 
                                 stallCount, maxStallCount, leftChange, rightChange);
                } else {
                    stallCount = 0;
                    isStalled = false;
                }
                
                // If stalled for several consecutive checks, abort the movement
                if (stallCount >= maxStallCount) {
                    Serial.println("Motor stall detected! Aborting movement.");
                    moveLeftMotor(0);
                    moveRightMotor(0);
                    return;
                }
                
                previousLeftPosition = currleft;
                previousRightPosition = currright;
                lastStallCheck = millis();
            }

            if(leftMotorSpeed > 30 || rightMotorSpeed > 30) {
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

            // For debugging
            // Serial.printf("left motor speed: %lf\tright motor speed: %lf\n", leftMotorSpeed, rightMotorSpeed);
        }

        moveLeftMotor(0);
        moveRightMotor(0);
        Serial.println("Done");
    }