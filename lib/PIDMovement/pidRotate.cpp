#include "pidRotate.h"

void turnTo(char direction)
{
    double targetDirection = 0;
    double currentAngle = getAngle();
    double error = 0;
    double kp = 0.6;
    double kd = 0.1;

    double previousError = 0;
    double derivative = 0;
    double previousTime = micros();
    double dt = 0;

    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;

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
    double sampleTime = micros();
    double sampleAngle = getAngle();

    error = targetDirection - getAngle();

    while (abs(error) > 0.5) 
    {
        if(micros() > sampleTime + 50e3) {
            if(sampleAngle == getAngle()) {
                stopMotors();
                return;
            }
            sampleTime = micros();
            sampleAngle = getAngle();
        } 

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

        dt = (micros() - previousTime) / 1e6;
        derivative = (error - previousError) / dt;

        leftMotorSpeed = (kp * error) + (kd * derivative);
        rightMotorSpeed = (kp * -error) + (kd * -derivative);

        Serial.printf("Left Motor Speed: %lf\t Right Motor Speed: %lf\t Error: %lf\n", leftMotorSpeed, rightMotorSpeed, error);

        moveLeftMotor(leftMotorSpeed);
        moveRightMotor(rightMotorSpeed);

        previousError = error;
        previousTime = micros();

    }

    stopMotors();
    delay(50);
}
