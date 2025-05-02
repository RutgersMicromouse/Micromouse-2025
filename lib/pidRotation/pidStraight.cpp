#include "pidStraight.h"
#include "pidRotate.h"
#define DIA 32
#define PI 3.1415926535897932384626433832795

double maxSpeed = 175;
double minSpeed = 100;
void straight(char direction, int distance)
{
    // ratio for the angle PID
    double AKp = 1.0;
    double AKd = 0.0;
    // ratios for the Encoder PID
    double LEKp = 0.85;
    double LEKd = 0.0;
    double REKp = 0.85;
    double REKd = 0.0;

    // Angle variables
    double AngleError = 0.0;
    double PreviousAngleError = 0.0;
    double currentAngle = angle();
    double targetDirection = 0.0;
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

    // encoder values
    double numTicks = (102 * distance) / (DIA * PI); // Num ticks that we need to travel
    double EncoderLeftError = numTicks;
    double EncoderRightError = numTicks;

    // clear the encoder count
    encLeft.clearCount();
    encRight.clearCount();

    // final motor speeds
    double LeftMotorSpeed = 0.0;
    double RightMotorSpeed = 0.0;

    // time values (for derivative)
    unsigned long PreviousTime = millis();
    unsigned long CurrentTime = millis();
    
    // Variables for stall detection
    unsigned long stallStartTime = 0;
    bool potentialStall = false;
    double prevLeftSpeed = 0;
    double prevRightSpeed = 0;
    const double stallThreshold = 3.0;        // Motor speed difference threshold
    const unsigned long stallTimeThreshold = 500; // 0.5 seconds in milliseconds

    // Variables for destination check
    bool isAtDestination = false;
    const double encoderTolerance = 5.0;      // Consider arrived if within this many ticks
    
    double basespeed = 135;
    while (1)
    {
        // retrieve the angle error
        AngleError = targetDirection - angle(); // figure out the AngleError
        while (AngleError > 180) {
            AngleError -= 360;
        }
        while (AngleError <= -180) {
            AngleError += 360;
        }
        
        // Calculate encoder errors
        EncoderLeftError = numTicks - getLeftEncoder();
        EncoderRightError = numTicks + getRightEncoder();
        
        // Check if we've reached the destination
        if (EncoderLeftError <= encoderTolerance && EncoderRightError <= encoderTolerance) {
            Serial.println("Reached destination");
            break;
        }
        
        // Constrain encoder errors for speed calculation
        EncoderLeftError = constrain(EncoderLeftError, 0, basespeed);
        EncoderRightError = constrain(EncoderRightError, 0, basespeed);
        
        // Calculate motor speeds
        LeftMotorSpeed = (LEKp * EncoderLeftError) + (AKp * AngleError);
        RightMotorSpeed = (REKp * EncoderRightError) - (AKp * AngleError);

        // Constrain speeds to valid range
        LeftMotorSpeed = constrain(LeftMotorSpeed, 0, maxSpeed);
        RightMotorSpeed = constrain(RightMotorSpeed, 0, maxSpeed);
        
        Serial.printf("left: %lf\tright: %lf\n", LeftMotorSpeed, RightMotorSpeed);
        
        // Check for stall condition
        if (fabs(LeftMotorSpeed - prevLeftSpeed) < stallThreshold && 
            fabs(RightMotorSpeed - prevRightSpeed) < stallThreshold) {
            if (!potentialStall) {
                // First time detected potential stall
                stallStartTime = millis();
                potentialStall = true;
                Serial.println("Potential stall detected");
            } else if ((millis() - stallStartTime) > stallTimeThreshold) {
                // Stall condition has persisted for enough time
                Serial.println("Stall confirmed - terminating movement");
                break;
            }
        } else {
            // Reset stall detection
            potentialStall = false;
        }

        // Update previous speeds for next iteration
        prevLeftSpeed = LeftMotorSpeed;
        prevRightSpeed = RightMotorSpeed;
        
        // Apply motor speeds
        moveLeftMotor(LeftMotorSpeed);
        moveRightMotor(RightMotorSpeed * 1.1);
        
        // Update time for next iteration
        PreviousTime = CurrentTime;
        CurrentTime = millis();
    }
    
    // Stop motors
    moveLeftMotor(0);
    moveRightMotor(0);
    delay(100);
    turnTo(direction);
}