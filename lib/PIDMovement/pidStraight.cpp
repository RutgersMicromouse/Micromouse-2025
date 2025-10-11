#include "pidStraight.h"
#include "pidRotate.h"
#define DIA 28
#define PI 3.1415926535897932384626433832795
#define leftTick 147 //273 for one block 147
#define rightTick 133 //257 for one block 133

void straight(char direction, int distance)
{
    encLeft.clearCount();
    encRight.clearCount();

    double leftP = 1; //1
    double rightP = 0.85; //0.85
    double encoderKd = 0.03; //0.05
    double leftAngleP = 0.8; //0.8
    double rightAngleP = 4; //4

    if (distance < 100) {
        // PID gains
        leftP = 1;
        rightP = 1; //1
        encoderKd = 0.15; //0.15
        leftAngleP = 0; //0
        rightAngleP = 0; //0
    }


    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    // Target ticks
    double leftNumTicks = (leftTick * distance) / (DIA * PI);
    double rightNumTicks = (rightTick * distance) / (DIA * PI);
      const double leftScale = 0.9;

    // PWM mapping
    double leftM = 150 / (leftNumTicks);
    double rightM = 150 / (rightNumTicks);

    // Error tracking
    double previousLeftError = leftNumTicks - (getLeftEncoder() * leftScale);
    double previousRightError = rightNumTicks - getRightEncoder();
    double currentLeftError = previousLeftError;
    double currentRightError = previousRightError;

    // Derivatives
    double leftEncoderDeriv = 0;
    double rightEncoderDeriv = 0;
    double previousTime = micros();
    double totalTime = micros();

    // Direction setup
    double angleError = 0.0;
    double targetDirection = 0.0;

    switch (direction) {
        case 'N': targetDirection = 0; break;
        case 'S': targetDirection = 180; break;
        case 'E': targetDirection = 90; break;
        case 'W': targetDirection = 270; break;
    }

    // ✅ Movement timeout setup
    const unsigned long movementTimeout = 2e6; // 5 seconds in microseconds
    unsigned long startTime = micros();

    // ✅ Stall detection setup
    const unsigned long stallTimeThreshold = 5e5; // 1 second without movement
    unsigned long lastEncoderMoveTime = startTime;
    long lastLeftEncoder = getLeftEncoder();
    long lastRightEncoder = getRightEncoder();

    while ((abs(previousLeftError) > 10) || (abs(previousRightError) > 10))
    {

        totalTime = micros();

        if ((totalTime - startTime) > movementTimeout) {
            Serial.println("Timeout reached — possible stall. Stopping motors.");
            break;
        }

        // ✅ Check for stall based on encoder movement
        if (abs(getLeftEncoder() - lastLeftEncoder) > 5 || abs(getRightEncoder() - lastRightEncoder) > 5) {
            lastEncoderMoveTime = totalTime;  
            lastLeftEncoder = getLeftEncoder();
            lastRightEncoder = getRightEncoder();
        }

        if ((totalTime - lastEncoderMoveTime) > stallTimeThreshold) {
            Serial.println("Encoder not moving — possible stall. Stopping motors.");
            break;
        }

        if(wallBrake() < 80) {
            moveLeftMotor(0);
            moveRightMotor(0);
            break;
        }

        // PID calculations
        angleError = targetDirection - getAngle();
        while (angleError > 180) angleError -= 360;
        while (angleError <= -180) angleError += 360;

        currentLeftError = leftNumTicks - (getLeftEncoder() * leftScale);
        currentRightError = rightNumTicks - getRightEncoder();
        

        leftEncoderDeriv = (currentLeftError - previousLeftError) / ((totalTime - previousTime) / 1e6);
        rightEncoderDeriv = (currentRightError - previousRightError) / ((totalTime - previousTime) / 1e6);

        leftSpeed = (leftP * leftM * currentLeftError) + (encoderKd * leftEncoderDeriv) + (leftAngleP * angleError);

        rightSpeed = (rightP * rightM * currentRightError) + (encoderKd * rightEncoderDeriv) - (rightAngleP * angleError);
         Serial.printf("Left speed: %lf right speed: %lf\n", leftSpeed, rightSpeed);

        // Send to motors
        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        previousLeftError = currentLeftError;
        previousRightError = currentRightError;
        previousTime = totalTime;

        delayMicroseconds(10);
    }

    // Stop motors
    stopMotors();
    delay(50);
}