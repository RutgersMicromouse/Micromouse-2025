#include "pidrotate.h"

double Kp = 2.7;
double Ki = 0;
double Kd = 0;



void turnTo(double goal_angle) {
    // Serial.println("Hello pidRotate!");

    double t_old = micros();
    
    double error = goal_angle - angle();
    // angle wrapping
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    double error_int;
    double error_deriv;

    double error_old = error;
    
    double angleOut;
    
    // sampling to check for motor stalling
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    while (true) {
        // Guard Clauses:
        // 1. At the destination angle
        if (abs(error) <= 1.0) { setRightPWM(0); setLeftPWM(0); return; }
        
        // 2. Stall Condition, 0.1 second
        if (micros() > sampleTime + 1e5){
           if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { setRightPWM(0); setLeftPWM(0); return; }

           // Update stall sampler
           sampleTime = micros();
           sampleRight = encRight.read();
           sampleLeft = encLeft.read();
           // Serial.printf("Angle error: %f\n", error);

        }
        //3. Spins for 5s (maybe?)
        // if (angl == 0 && micros() > sampleTime + 1e6) { setRightPWM(0); setLeftPWM(0); return; } // IMU error
        //End Guard Clauses

        // // Serial.println("Hello error loop!");
        error = goal_angle - angle();
        if (error < -180.0) {error += 360;} else if (error > 180) {error -= 360;}
        error_int = error * (micros() - t_old);
        error_deriv = (error - error_old)/(micros() - t_old);

        angleOut = Kp * error + Ki * error_int + Kd * error_deriv;
        setLeftPWM(angleOut); setRightPWM(-angleOut); 

        // update error_angle_old, and t_old
        error_old = error; t_old = micros();

        // // Serial.printf("Angle error: %f\n", error);
    }
}