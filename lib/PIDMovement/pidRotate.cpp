#include "pidRotate.h"

void turnTo(char direction)
{
    double targetDirection = 0;
    double currentAngle = getAngle();
    double error = 0;
    double totalError = 0;

    double kp = 0.6;

    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;

    // double totalTime = micros();
    // double previousTime = micros();
    
    // New variables for position stability check
    unsigned long stablePositionStartTime = 0;
    bool isStable = false;
    double lastError = 0;
    const double errorThreshold = 1.0;                // Consider position stable if error is less than this
    const unsigned long stableTimeThreshold = 500000; // 0.5 seconds in microseconds

    // Variables for stall detection
    unsigned long stallStartTime = 0;
    bool potentialStall = false;
    double prevLeftSpeed = 0;
    double prevRightSpeed = 0;
    const double stallThreshold = 3.0;               // Motor speed difference threshold
    const unsigned long stallTimeThreshold = 500000; // 0.5 seconds in microseconds

    // Based on direction we want to go to, we set the target angle
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

    double startTime = micros();
    double sampleAngle = getAngle();

    while (1)
    {
        currentAngle = getAngle();
        error = targetDirection - currentAngle;

        while (error > 180)
        {
            error -= 360;
        }
        while (error < -180)
        {
            error += 360;
        }

        leftMotorSpeed = kp * error;
        rightMotorSpeed = kp * -error;

        // Check for stall condition
        if (fabs(leftMotorSpeed - prevLeftSpeed) < stallThreshold && 
            fabs(rightMotorSpeed - prevRightSpeed) < stallThreshold) {
            if (!potentialStall) {
                // First time detected potential stall
                stallStartTime = micros();
                potentialStall = true;
            } else if ((micros() - stallStartTime) > stallTimeThreshold) {
                // Stall condition has persisted for enough time
                Serial.println("Stall detected - terminating turn");
                break;
            }
        } else {
            // Reset stall detection
            potentialStall = false;
        }

        // Update previous speeds for next iteration
        prevLeftSpeed = leftMotorSpeed;
        prevRightSpeed = rightMotorSpeed;

        // Original position stability check (if needed)
        if (fabs(error) < errorThreshold) {
            if (!isStable) {
                stablePositionStartTime = micros();
                isStable = true;
            } else if ((micros() - stablePositionStartTime) > stableTimeThreshold) {
                // Position has been stable for required time
                break;
            }
        } else {
            isStable = false;
        }

        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);
    }

    moveLeftMotor(0);
    moveRightMotor(0);
    delay(1);
    
}