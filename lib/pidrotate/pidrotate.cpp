#include "pidrotate.h"

double Kp = 2.4;
double Ki = 0;
double Kd = 0;



void turnTo(double goal_angle) {
    Serial.println("Hello pidRotate!");
    double angl = angle();
    double t_old = micros();
    double out;
    double error_int;
    double error_deriv;
    double error_old = goal_angle - angl;
    double error =goal_angle - angl;
    // sampling to check for motor stalling
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();
    while (true) {
        angl = angle();
        Serial.println(error);
        // Guard Clauses:
        // 1. At the destination angle
        if (abs(error) <= 1.0) { setRightPWM(0); setLeftPWM(0); return; }
        // 2. Stall Condition
        if (micros() > sampleTime + 50e5){
           if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2)
                                  { setRightPWM(0); setLeftPWM(0); return; };
        }
        //3. Spins for 5s (maybe?)
        if (angl == 0 && micros() > sampleTime + 1e6) { setRightPWM(0); setLeftPWM(0); return; } // IMU error
        //End Guard Clauses

        Serial.println("Hello error loop!");
        error = goal_angle - angl;
        if (error < -180.0) {error += 360;} else if (error > 180) {error -= 360;}
        error_int = error * (micros() - t_old);
        error_deriv = (error - error_old)/(micros() - t_old);

        double angle_out = Kp * error + Ki * error_int + Kd * error_deriv;
        setLeftPWM(angle_out); setRightPWM(-angle_out); 
        // update error_dist_old, error_angle_old, and t_old
        error_old = error; t_old = micros();
        //delayMicroseconds(0); // Additional thing to change if IMU goofs up
        Serial.printf("%f , %c", angl, error);
    };}