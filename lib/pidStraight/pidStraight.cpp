#include "pidStraight.h"
#include "imu.h"
#include "motors.h"
#define PI 3.1415926535897932384626433832795
#define DIA 32

double P = 0.2;
double I = 0;
double D = 0;


void setForwardPWM(double distance) {
    //Calculating the num of ticks needed to move the motors for our final position
    distance = distance * 1.35;
    double rotations = distance / (PI * DIA);
    //Serial.println(rotations);
    int ticks = 360 * rotations;

    //Calculating the final position that we need to be for the different motors
    long right_final_pos = ticks + encRight.read();
    long left_final_pos = ticks +  encLeft.read();

    //Old error positions before we move our motors
    long old_right_pos_error = right_final_pos - encRight.read();
    long old_left_pos_error = left_final_pos - encLeft.read();

    long startTime = micros();
    double right_error_integral = 0;
    double left_error_integral = 0;

    double angle_goal;

    double currentAngle = angle();
        if(currentAngle < 45 || currentAngle > 315) {
            angle_goal = 0;
        } else if(currentAngle > 45 && currentAngle < 135) {
            angle_goal = 90;
        } else if(currentAngle > 135 && currentAngle < 225) {
            angle_goal = 180;
        } else {
            angle_goal = 270;
        }

    long sampleTime = micros();
    long sampleRight = encRight.read();
    long sampleLeft = encLeft.read();

    while(abs(encRight.read() - right_final_pos) > 5 && abs(encLeft.read() - left_final_pos) > 3) {
        currentAngle = angle();

        long new_right_error = right_final_pos - encRight.read();
        long new_left_error = left_final_pos - encLeft.read(); 

        //Calculation of time for PID equation
        long currentTime = micros();
        long dt = currentTime - startTime;
        
        //Integral (I) 
        if(abs(new_right_error) < 200) {      
            right_error_integral += ((new_right_error - old_right_pos_error) * dt) / 2;
            left_error_integral += ((new_left_error - old_left_pos_error) * dt) / 2;
        } 

        //Derivative (D)
        long right_error_deriv = (new_right_error - old_right_pos_error)/dt;
        long left_error_deriv = (new_left_error - old_left_pos_error)/dt;

        if(dt == 0) {
            right_error_deriv = 0;
            left_error_deriv = 0;
        }

        //Finding out error angle and angle wrapping
        double angle_error = angle_goal - currentAngle;
        if (angle_error < -180) angle_error += 360;
        if (angle_error > 180) angle_error -= 360; 

        // Serial.println(angle_error);
        // delay(1000);

        //To deal with stalling and check for front wall
        if(micros() > sampleTime + 50e3) {
            if(abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) {
                setRightPWM(0);
                setLeftPWM(0);
                return;
            }

            if(front() < 15) {
                setRightPWM(0);
                setLeftPWM(0);
                return;
            }
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
            sampleTime = micros();
        }
    

        //Setting the left and right PWM values given the PID equation
        int rightpid =  (P * new_right_error) + (I * right_error_integral) + (D * right_error_deriv) - (2 * angle_error);
        int leftpid = (P * new_left_error) + (I * left_error_integral) + (D * left_error_deriv) + (2 * angle_error);

        // if(leftpid > 150){
        //     leftpid = 150;
        // }
        // else if(leftpid < -150){
        //     leftpid = -150;
        // }

        // if(rightpid > 150){
        //     rightpid = 150;
        // }
        // else if(rightpid < -150){
        //     rightpid = -150;
        // }

        setRightPWM(rightpid );
        setLeftPWM( leftpid * 1.01);

        old_right_pos_error = new_right_error;
        old_left_pos_error = new_left_error;
        startTime = currentTime;

    }
    
    //Stopping the robot after reaching destination
    setRightPWM(0);
    setLeftPWM(0);

    
}