#include "pidStraight.h"
#include "pidRotate.h"

#define rightSensor 3
#define leftSensor 4

int16_t leftWallDistance();
int16_t rightWallDistance();
double getDistError();

void straight(char direction, int distance)
{
    
    double baseSpeed = 50;

    double Km = 0.5;
    double Td = 0.75;

    double runTime = 0.605e6; // microseconds

    // deceleration rate (PWM per second)
    double decelRate = 100;

    // time to go from baseSpeed -> 0
    double decelTime = baseSpeed / decelRate;

    double decelTime_us = decelTime * 1e6;

    // shorten constant phase so distance stays identical
    double constantTime = runTime - decelTime_us/2;

    double startTime = micros();
    double currentTime = micros();
    double oldTime = micros();

    double timeElapsed = 0;

    double currentError = getDistError();
    double oldError = currentError;

    while(timeElapsed < constantTime + decelTime_us)
    {
        currentTime = micros();
        timeElapsed = currentTime - startTime;

        double speed = baseSpeed;

        // DECELERATION PHASE
        if(timeElapsed > constantTime)
        {
            double t = (timeElapsed - constantTime)/1e6;

            speed = baseSpeed - decelRate * t;

            if(speed < 0)
                speed = 0;
        }

        // PID WALL FOLLOW
        currentError = getDistError();

        double dt = (currentTime - oldTime)/1e6;
        if(dt <= 0) dt = 0.001;

        double derError = (currentError - oldError)/dt;

        double correction = Km*currentError + Td*Km*derError;

        double leftSpeed = speed + correction;
        double rightSpeed = speed - correction;

        leftSpeed = constrain(leftSpeed,0,180);
        rightSpeed = constrain(rightSpeed,0,180);

        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        oldError = currentError;
        oldTime = currentTime;

        printf("Speed:%lf  L:%lf  R:%lf\n",speed,leftSpeed,rightSpeed);
    }

    moveLeftMotor(0);
    moveRightMotor(0);
    delay(100);
}
/*
void straight(char direction, int distance)
{
    double baseSpeed = 50;

    double leftSpeed = baseSpeed;
    double rightSpeed = baseSpeed;

    double runTime = 0.605e6; // microseconds

    double startTime = micros();
    double currentTime = micros();
    double oldTime = micros();

    double timeElapsed = 0;

    double Km = 0.5;
    double Td = 0.75;

    double currentError = getDistError();
    double oldError = currentError;

    // deceleration parameters
    double decelRate = 50.0; // PWM/sec
    double decelStartTime = runTime * 0.8; // start deceleration at 80%

    while(timeElapsed < runTime)
    {
        currentTime = micros();
        timeElapsed = currentTime - startTime;

        // PID error
        currentError = getDistError();

        double dt = (currentTime - oldTime) / 1e6;

        if(dt <= 0) dt = 0.001;

        double derError = (currentError - oldError) / dt;

        // Base speed calculation
        double speed = baseSpeed;

        // Deceleration
        if(timeElapsed > decelStartTime)
        {
            double t = (timeElapsed - decelStartTime) / 1e6; // seconds
            speed = baseSpeed - decelRate * t;

            if(speed < 0)
                speed = 0;
        }

        // PID correction
        double correction = Km * currentError + (Td * Km) * derError;

        leftSpeed = speed + correction;
        rightSpeed = speed - correction;

        leftSpeed = constrain(leftSpeed, 0, 180);
        rightSpeed = constrain(rightSpeed, 0, 180);

        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        oldError = currentError;
        oldTime = currentTime;

        printf("L:%lf R:%lf Time:%lf\n", leftSpeed, rightSpeed, timeElapsed);
    }

    moveLeftMotor(0);
    moveRightMotor(0);

    delay(100);
}
*/
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
    int16_t leftDist = leftWallDistance();
    int16_t rightDist = rightWallDistance();

    return (double)(rightDist - leftDist);
}
/*
#include "pidStraight.h"
#include "pidRotate.h"
#define DIA 28
#define PI 3.1415926535897932384626433832795

#define rightSensor 3
#define leftSensor 4

int16_t leftWallDistance();
int16_t rightWallDistance();
double getDistError();


void straight(char direction, int distance)
{

    /*

        150 starting speed
        Getting distance from both sensors
            subtract it (R - L)
            == 0 --> keep speed the same
            > 0 --> closer to the left wall (R > L) --> increase left speed/decrease right speed
            < 0 --> closer to the right wall (L > R) --> increase right speed/decrease left speed
        Distance difference to a PWM value (adding or subtract to the 180)
        Doing it based on time vs distance (Adjust time as needed)

        PID Tuning:
            - Use Kp, Ki = (1/Ti) * Kp, Kd = Td * Kp

    
    

    //Figure out starting speed for both motors (IMPORTANT)

    double baseSpeed = 50;
    double leftBaseSpeed, rightBaseSpeed;
    double runTime = 0.605e6;
    double tick = runTime/100;
    double tick_dist = baseSpeed*tick;

    double leftSpeed, rightSpeed;

    double startTime = micros();
    double oldTime = micros();
    double currentTime = micros();
    double timeElapsed = currentTime - startTime;
    double slowDownTime = 0.8 * runTime;

    double distError = getDistError();
    double oldError = distError, currentError = oldError;

    double Km = 0.5, Td = 0.75;
    double Dr = 0.5;
    double decelerationTime = (0-baseSpeed)/-Dr;
    double decelerationDist = (baseSpeed*decelerationTime) + (0.5*(-Dr*(decelerationTime*decelerationTime)));
    double tick_dec_travel = decelerationDist/tick_dist;

    double constant_speed_time = runTime-(decelerationTime*1000);
    printf("decelerationTime: %lf     \n", decelerationTime);
    printf("decelerationDist: %lf     \n", decelerationDist);
    printf("tick_dec_travel: %lf     \n", tick_dec_travel);
    printf("constant_speed_time: %lf     \n", constant_speed_time);

    while(timeElapsed < constant_speed_time+decelerationTime) {
        
        if(timeElapsed > constant_speed_time) {
            printf("we are here!");
            leftSpeed -= baseSpeed*Dr;
            rightSpeed -= baseSpeed*Dr;
            
            //moveLeftMotor(0);
            //moveRightMotor(0);
            //break;
        }
        //Get the current wall error
        currentError = getDistError();

        //Getting time and calculating dt for derivative calculation
        currentTime = micros();
        double dt = (currentTime - oldTime)/(1e6);

        double derError = (currentError - oldError) / dt;

       //Doing speed calculations
       leftSpeed = baseSpeed + Km * currentError + (Td * Km) * derError;
       rightSpeed = baseSpeed - Km * currentError + (Td * Km) * derError;

       leftSpeed = constrain(leftSpeed, 0, 180);
       rightSpeed = constrain(rightSpeed, 0, 180);

        //Moving the motorss
        moveLeftMotor(leftSpeed);
        moveRightMotor(rightSpeed);

        //Figuring out the error
        oldError = currentError;

        oldTime = currentTime;
        currentTime = micros();
        timeElapsed = currentTime - startTime;

        printf("Left Motor Speed: %lf     ", leftSpeed);
        printf("Right Motor Speed: %lf \n", rightSpeed);
        printf("timeElapsed: %lf \n", timeElapsed);


    }

    moveLeftMotor(0);
    moveRightMotor(0);
    delay(100);    
}

int16_t leftWallDistance() {

    int16_t leftTime = pulseIn(leftSensor, HIGH);
    int16_t leftDistance = (leftTime - 1000) * 3 / 4;

    return leftDistance/10;

}

int16_t rightWallDistance() {
    
    int16_t rightTime = pulseIn(rightSensor, HIGH);
    int16_t rightDistance = (rightTime - 1000) * 3 / 4;
    
    return rightDistance/10;
}

double getDistError() {

    int16_t leftDist = leftWallDistance();
    int16_t rightDist = rightWallDistance();

    double error = (double) (rightDist - leftDist);

    return error;


}
*/