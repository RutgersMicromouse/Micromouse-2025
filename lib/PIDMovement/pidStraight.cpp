#include "pidStraight.h"
#include "pidRotate.h"
#define DIA 28
#define PI 3.1415926535897932384626433832795
#define leftTick 140
#define rightTick 140

void straight(char direction, int distance)
{
    distance = distance * 1.05;
    encLeft.clearCount();
    encRight.clearCount();

    // PID gains
    double leftP = 1;
    double rightP = 0.95;
    double encoderKd = 0.5;
    double leftAngleP = 1.3; 
    double rightAngleP = 1.3; 

    // Motor speeds
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    // Target ticks
    double leftNumTicks = (leftTick * distance) / (DIA * PI);
    double rightNumTicks = (rightTick * distance) / (DIA * PI);

    // PWM mapping
    double leftM = 125 / (leftNumTicks);
    double rightM = 125 / (rightNumTicks);

    // Error tracking
    double previousLeftError = leftNumTicks - getLeftEncoder();
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

    while (abs(leftNumTicks - getLeftEncoder()) > 10 &&abs(rightNumTicks - getRightEncoder()) > 10)
    {
        totalTime = micros();

        // ✅ Check for total movement timeout
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

        // PID calculations
        angleError = targetDirection - getAngle();
        while (angleError > 180) angleError -= 360;
        while (angleError <= -180) angleError += 360;

        currentLeftError = leftNumTicks - getLeftEncoder();
        currentRightError = rightNumTicks - getRightEncoder();

        leftEncoderDeriv = (currentLeftError - previousLeftError) / (totalTime - previousTime);
        rightEncoderDeriv = (currentRightError - previousRightError) / (totalTime - previousTime);

        leftSpeed = (leftP * leftM * currentLeftError) + (encoderKd * leftEncoderDeriv) + (leftAngleP * angleError);

        rightSpeed = (rightP * rightM * currentRightError) + (encoderKd * rightEncoderDeriv) - (rightAngleP * angleError);

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
    delay(250);
}
