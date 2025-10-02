#include "pidStraight.h"
#include "pidRotate.h"
#define DIA 28
#define PI 3.1415926535897932384626433832795
#define leftTick 285
#define rightTick 395

void straight(char direction, int distance)
{

    /*For the angle:

    */

    //Clearing encoder counts
    encLeft.clearCount();
    encRight.clearCount();

    //PID Variables
        /*Start at 1 and then go to 0.1 and all the way to 10 (larger value should mean there is a smaller error)
          Integral will make us reach the target faster, but might overshoot/become unstable
          Smaller error, means the PWM value should be smaller (larger error means the PWM value should be larger)
        */
    double leftP = 0.75;
    double rightP = 0.75;

    //Motor Speeds
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    //Calculating number of ticks needed to move
    //518.39 for 1 block (left wheel), 718 for 1 block (right wheel)
    double leftNumTicks = (leftTick * distance) / (DIA * PI);
    double rightNumTicks = (rightTick * distance) / (DIA * PI);

    //Mapping ticks to PWM (Will result in speeds being between 0 and 255) 
    double leftMaxError = leftNumTicks;
    double rightMaxError = rightNumTicks;
    double minError = 5;
    double leftM = 255/(leftMaxError - minError); //Equation: (255 - 0)/(leftMaxError - minError)
    double rightM = 255/(rightMaxError - minError); //Equation: (255 - 0)/(rightMaxError - minError)

    double previousLeftError = leftNumTicks - getLeftEncoder();
    double previousRightError = rightNumTicks - getRightEncoder();
    double currentLeftError = previousLeftError;
    double currentRightError = previousRightError;
    
    while (abs(currentLeftError) > 5 || abs(currentRightError) > 5)
    {

        //Getting the new errors for the left and right
        currentLeftError = leftNumTicks - getLeftEncoder();
        currentRightError = rightNumTicks - getRightEncoder();

        // Calculate motor speeds
        leftSpeed = (leftP * leftM * currentLeftError);
        rightSpeed = (rightP * rightM * currentRightError);

        //Running the motors
        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        //Getting the new encoder values and updating the error
        previousLeftError = currentLeftError;
        previousRightError = currentRightError;

        delayMicroseconds(10);

    }
    
    // Stop motors
    stopMotors();

    
}