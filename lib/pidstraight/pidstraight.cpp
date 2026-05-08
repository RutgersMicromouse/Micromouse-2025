#include "pidstraight.h"
#include <pidrotate.h>
#include <sidedist.h>

#define rightSensor 3
#define leftSensor 4

double getDistError();

double baseTime = .35e6; //400e6 --> 5/7 //0.35 for 20 //
double halfBlock = 90;
//double blockLength = 160;

double Kp_angle = 0.8;
double Ki_angle = 0.0;
double Kd_angle = 0.1;

double identity_diag[8] = {0, 45, 90, 135, 180, 225, 270, 315};
void pidForwardSetup() {
    // any setup for pid forward can go here
}

//Center --> 4cm
//Max distance from other sensor while hitting opposite wall --> 8 cm
void pidForward(double distance)
{
    
    double baseSpeed = 100; //50 PWM --> 0.605 seconds
    double Km = 0.75;//0.5;
    double Td = 0.4;//0.75;
    double Ka = 0.25;

    double runTime = baseTime;
    //This if statement is for 1/2 block
    if(distance == halfBlock) {
        runTime = runTime * 0.35;
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
    double currentAngle = angle();

    // --- Dynamic IMU Angle Mapping ---
    int closest_index = 0;
    double arr_diag[8];

    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - currentAngle;
        if (arr_diag[i] > 180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) {
            closest_index = i;
        }
    }
    angle_goal = identity_diag[closest_index];
    // ---------------------------------

    int Wall_threshold = 8;

    double speed = baseSpeed;

    while((timeElapsed < (constantRatio * runTime) + decelTime))
    {

        //Checking the front to not crash
        if(front() < 60) {
            setLeftPWM(0);
            setRightPWM(0);
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
        currentAngle = angle();

        double dt = (currentTime - oldTime)/1e6;
        if(dt <= 0) dt = 0.001;

        double derError = (currentError - oldError)/dt;
        double angleError = angle_goal - currentAngle;
        if (angleError < -180) angleError += 360;
        if (angleError > 180) angleError -= 360; 
        double correction = Km * currentError + Td*Km*derError + Ka*angleError;

        double leftSpeed = speed + correction;
        double rightSpeed = speed - correction;

        setLeftPWM(leftSpeed);
        setRightPWM(rightSpeed);

        oldError = currentError;
        oldTime = currentTime;

        // Serial.printf("Speed: %lf, %lf\n", leftSpeed, rightSpeed);


    }

    if(distance != 180) {
        setLeftPWM(0);
        setRightPWM(0);
    } else {
        setLeftPWM(20);
        setRightPWM(20);
    }
    delay(100);

}

double getDistError()
{
    static double heldError = 0;   // remembers last good error

    int16_t leftDist  = getLeftSideDist();
    int16_t rightDist = getRightSideDist();

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



void pidForwardLeftWallFollow() {
    
    // Find the closest world angle axis
    double goal_angle;
    int closest_index = 0;
    double arr_diag[8];

    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - angle();
        if (arr_diag[i] > 180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) {
            closest_index = i;
        }
    }
    goal_angle = identity_diag[closest_index];

        
    double t_old = micros();
    double error_angle = goal_angle - angle();
    if (error_angle > 180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;

    double error_int_angle;
    double error_deriv_angle;
    double error_angle_old = error_angle;
    double angleOut;

    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    while (true) {
        if (!leftWall()) {
            delay(120);
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }

        if (micros() > sampleTime + 1e5) {
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) {
                setRightPWM(0);
                setLeftPWM(0);
                return;
            }
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
        }

        if (front() < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }

        error_angle = goal_angle - angle();
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        error_int_angle += error_angle * (micros() - t_old);
        error_deriv_angle = (error_angle - error_angle_old) / (micros() - t_old);

        angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
        setLeftPWM(125 - angleOut);
        setRightPWM(125 + angleOut);

        error_angle_old = error_angle;
        t_old = micros();
    }
}