#include "pidStraight.h"
#include "pidRotate.h"
#include "../IMU/imu.h"
#include "../distanceSensors/distanceSensors.h"

#define rightSensor 3
#define leftSensor 4

int16_t leftWallDistance();
int16_t rightWallDistance();
double getDistError();

double baseTime = .370e6; //50 pwm --> 0.605
double blockLength = 160;

//Center --> 4cm
//Max distance from other sensor while hitting opposite wall --> 8 cm
void straight(char direction, int distance)
{
    
    double baseSpeed = 85; //50 PWM --> 0.605 seconds
    double Km = 0.75;//0.5;
    double Td = 0.30;//0.75;
    double Ka = 0.25;

    double runTime = baseTime;

    if(distance == 80) {
        runTime = runTime * 0.4;
    }

    if(distance == 100) {
        runTime = runTime * 0.60;
    }

    double constantRatio = 0;
    double decelTime = 2 * runTime * (1 - constantRatio);
    double decelRate = -(baseSpeed/decelTime);

    double startTime = micros();
    double currentTime = micros();
    double oldTime = micros();

    double timeElapsed = 0;

    double currentError = getDistError();
    double oldError = currentError;

    double angle_goal;

    double currentAngle = getAngle();
    char goalDirection;

        if(currentAngle >= 337.5 || currentAngle < 22.5) {
        angle_goal = 0;
        goalDirection = 'N';
    }
    else if(currentAngle >= 22.5 && currentAngle < 67.5) {
        angle_goal = 45;
        goalDirection = 'A'; // NE
    }
    else if(currentAngle >= 67.5 && currentAngle < 112.5) {
        angle_goal = 90;
        goalDirection = 'E';
    }
    else if(currentAngle >= 112.5 && currentAngle < 157.5) {
        angle_goal = 135;
        goalDirection = 'B'; // SE
    }
    else if(currentAngle >= 157.5 && currentAngle < 202.5) {
        angle_goal = 180;
        goalDirection = 'S';
    }
    else if(currentAngle >= 202.5 && currentAngle < 247.5) {
        angle_goal = 225;
        goalDirection = 'C'; // SW
    }
    else if(currentAngle >= 247.5 && currentAngle < 292.5) {
        angle_goal = 270;
        goalDirection = 'W';
    }
    else {
        angle_goal = 315;
        goalDirection = 'D'; // NW
    }

    int Wall_threshold = 8;

    double speed = baseSpeed;

    Serial.printf("Decel Time: %lf\n", decelTime);

    Serial.printf("Decel Rate: %lf\n", decelRate);

    Serial.printf("Total Time: %lf\n", (constantRatio*runTime)+decelTime);

    Serial.printf("Time Starts to Decelerate: %lf\n", (constantRatio*runTime));

    while((timeElapsed < (constantRatio * runTime) + decelTime))
    {

        //Checking the front to not crash
        if(checkFrontWall() < 60) {
            moveLeftMotor(0);
            moveRightMotor(0);
            break;
        }

        currentTime = micros();
        timeElapsed = currentTime - startTime;

        // DECELERATION PHASE
        if(timeElapsed > constantRatio * runTime)
        {
            speed = speed + (decelRate*(currentTime - oldTime));

            if(speed < 0)
                speed = 0;
        }

        // PID WALL FOLLOW
        currentError = getDistError();
        currentAngle = getAngle();

        double dt = (currentTime - oldTime)/1e6;
        if(dt <= 0) dt = 0.001;

        double derError = (currentError - oldError)/dt;
        double angleError = angle_goal - currentAngle;
        if (angleError < -180) angleError += 360;
        if (angleError > 180) angleError -= 360; 
        double correction = Km * currentError + Td*Km*derError + Ka*angleError;

        double leftSpeed = speed + correction;
        double rightSpeed = speed - correction;

        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        oldError = currentError;
        oldTime = currentTime;

        Serial.printf("Speed: %lf, %lf\n", leftSpeed, rightSpeed);


    }

    moveLeftMotor(0);
    moveRightMotor(0);
    delay(100);

}

void straightASTAR(char direction)
{
    
    double baseSpeed = 85; //50 PWM --> 0.605 seconds
    double Km = 0.75;//0.5;
    double Td = 0.30;//0.75;
    double Ka = 0.25;

    double currentTime = micros();
    double oldTime = micros();

    double currentError = getDistError();
    double oldError = currentError;

    double angle_goal;

    double currentAngle = getAngle();
    char goalDirection;

    if(currentAngle >= 337.5 || currentAngle < 22.5) {
        angle_goal = 0;
        goalDirection = 'N';
    }
    else if(currentAngle >= 22.5 && currentAngle < 67.5) {
        angle_goal = 45;
        goalDirection = 'A'; // NE
    }
    else if(currentAngle >= 67.5 && currentAngle < 112.5) {
        angle_goal = 90;
        goalDirection = 'E';
    }
    else if(currentAngle >= 112.5 && currentAngle < 157.5) {
        angle_goal = 135;
        goalDirection = 'B'; // SE
    }
    else if(currentAngle >= 157.5 && currentAngle < 202.5) {
        angle_goal = 180;
        goalDirection = 'S';
    }
    else if(currentAngle >= 202.5 && currentAngle < 247.5) {
        angle_goal = 225;
        goalDirection = 'C'; // SW
    }
    else if(currentAngle >= 247.5 && currentAngle < 292.5) {
        angle_goal = 270;
        goalDirection = 'W';
    }
    else {
        angle_goal = 315;
        goalDirection = 'D'; // NW
    }

    int Wall_threshold = 8;
    double speed = baseSpeed;


    while(checkFrontWall() < 130)
    {

        currentTime = micros();

        // PID WALL FOLLOW
        currentError = getDistError();
        currentAngle = getAngle();

        double dt = (currentTime - oldTime)/1e6;
        if(dt <= 0) dt = 0.001;

        double derError = (currentError - oldError)/dt;
        double angleError = angle_goal - currentAngle;
        if (angleError < -180) angleError += 360;
        if (angleError > 180) angleError -= 360; 
        double correction = Km * currentError + Td*Km*derError + Ka*angleError;

        double leftSpeed = speed + correction;
        double rightSpeed = speed - correction;

        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        oldError = currentError;
        oldTime = currentTime;

        Serial.printf("Speed: %lf, %lf\n", leftSpeed, rightSpeed);


    }

    moveLeftMotor(0);
    moveRightMotor(0);
    delay(100);

}

int16_t leftWallDistance()
{
    int16_t leftTime = pulseIn(leftSensor, HIGH);
    int16_t leftDistance = (leftTime - 1000) * 3 / 4;
    return leftDistance / 10;
}

int16_t rightWallDistance()
{
    int16_t rightTime = pulseIn(rightSensor, HIGH);
    int16_t rightDistance = (rightTime - 1000) * 3 / 4;
    return rightDistance / 10;
}

double getDistError()
{
    static double heldError = 0;   // remembers last good error

    int16_t leftDist  = leftWallDistance();
    int16_t rightDist = rightWallDistance();

    int WALL_THRESHOLD = 8;   // cm
    double TARGET_DIST = 4;   // center distance from wall

    bool leftWall  = leftDist  < WALL_THRESHOLD;
    bool rightWall = rightDist < WALL_THRESHOLD;

    if(leftWall && rightWall)
    {
        // normal centering
        heldError = rightDist - leftDist;
    }
    else if(leftWall)
    {
        // follow left wall
        heldError = TARGET_DIST - leftDist;
    }
    else if(rightWall)
    {
        // follow right wall
        heldError = rightDist - TARGET_DIST;
    }
    else
    {
        // no walls detected
        // hold previous error so robot continues straight
    }

    heldError = constrain(heldError, -8, 8);

    return heldError;
}

