#include "pidrotate.h"

double Kp = 1.2;
double Ki = 0.5;
double Kd = 0.0;
double mult = 1;

double Kp_half = 1.05;
double Ki_half = 0.05;
double Kd_half = 0.03;

/*
void turnTo(double goal_angle) {    
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
        if (abs(error) <= 0.5) { setRightPWM(0); setLeftPWM(0); delay(500); return; }
        
        // 2. Stall Condition, 0.1 second
        if (micros() > sampleTime + 1e5){
           if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { setRightPWM(20); setLeftPWM(20); return; }

           // Update stall sampler
           sampleTime = micros();
           sampleRight = encRight.read();
           sampleLeft = encLeft.read();
           
        }
        //3. Spins for 5s (maybe?)
        // if (angl == 0 && micros() > sampleTime + 1e6) { setRightPWM(0); setLeftPWM(0); return; } // IMU error
        //End Guard Clauses

        error = goal_angle - angle();
        if (error < -180.0) {error += 360;} else if (error > 180) {error -= 360;}
        error_int = error * (micros() - t_old);
        error_deriv = (error - error_old)/(micros() - t_old);

        angleOut = (Kp * error + Ki * error_int + Kd * error_deriv)* mult;
        setLeftPWM(angleOut); setRightPWM(-angleOut); 

        // update error_angle_old, and t_old
        error_old = error; t_old = micros();
        Serial.printf("Goal: %f, Current Angle: %f, Error: %f, Output: %f\n", goal_angle, angle(), error, angleOut);

    }

}
*/

const double iLimit = 50.0; 

void turnTo(double goal_angle) {    
    double basespeed = -7;
    unsigned long t_old = micros();
    double error_int = 0;
    double error_old = 0;
    
    // Initial error calculation for derivative start
    double start_error = goal_angle - angle();
    if (start_error > 180) start_error -= 360;
    if (start_error < -180) start_error += 360;
    error_old = start_error;

    // Stall Detection Variables
    unsigned long sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    while (true) {
        double current_angle = angle();
        double error = goal_angle - current_angle;

        // 1. Angle Wrapping (Shortest Path)
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // 2. Guard Clause: Target Reached
        if (abs(error) <= 0.5) { 
            setRightPWM(0); 
            setLeftPWM(0); 
            delay(200); // Short settle time
            return; 
        }

        // 3. Timing Calculation (Seconds)
        unsigned long now = micros();
        double dt = (now - t_old) / 1000000.0; 
        if (dt <= 0) dt = 0.001; // Safety against 0 divisor

        // 4. PID Logic
        // Integral with Windup Guard
        error_int += error * dt;
        if (error_int > iLimit) error_int = iLimit;
        if (error_int < -iLimit) error_int = -iLimit;

        // Derivative
        double error_deriv = (error - error_old) / dt;

        double angleOut = basespeed + (Kp * error + Ki * error_int + Kd * error_deriv) * mult;

        printf("Goal: %f, Current Angle: %f, Error: %f, Output: %f\n", goal_angle, angle(), error, angleOut);

        // 5. Output to Motors
        setLeftPWM(angleOut); 
        setRightPWM(-angleOut); 

        // 6. Stall Detection (Check every 0.1s)
        if (now > sampleTime + 100000) {
           if (abs(encRight.read() - sampleRight) < 2 && abs(encLeft.read() - sampleLeft) < 2) { 
               // If stalled, stop motors and exit to prevent overheating
               setRightPWM(0); 
               setLeftPWM(0); 
               return; 
           }
           sampleTime = now;
           sampleRight = encRight.read();
           sampleLeft = encLeft.read();
        }

        // 7. Update State
        error_old = error; 
        t_old = now;

        // Optional: Debugging
        // Serial.printf("Err: %.2f | Out: %.2f\n", error, angleOut);
        
        delay(5); // Small delay to prevent CPU hogging and IMU jitter
    }
}

void turnToHalf(double goal_angle) {
    
    unsigned long t_old = micros();
    double error_int = 0;
    double error_old = 0;
    
    // Initial error calculation for derivative start
    double start_error = goal_angle - angle();
    if (start_error > 180) start_error -= 360;
    if (start_error < -180) start_error += 360;
    error_old = start_error;

    // Stall Detection Variables
    unsigned long sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    while (true) {
        double current_angle = angle();
        double error = goal_angle - current_angle;

        // 1. Angle Wrapping (Shortest Path)
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        // 2. Guard Clause: Target Reached
        if (abs(error) <= 0.5) { 
            setRightPWM(0); 
            setLeftPWM(0); 
            delay(200); // Short settle time
            return; 
        }

        // 3. Timing Calculation (Seconds)
        unsigned long now = micros();
        double dt = (now - t_old) / 1000000.0; 
        if (dt <= 0) dt = 0.001; // Safety against 0 divisor

        // 4. PID Logic
        // Integral with Windup Guard
        error_int += error * dt;
        if (error_int > iLimit) error_int = iLimit;
        if (error_int < -iLimit) error_int = -iLimit;

        // Derivative
        double error_deriv = (error - error_old) / dt;

        double angleOut = (Kp_half * error + Ki_half * error_int + Kd_half * error_deriv) * mult;

        // 5. Output to Motors
        setLeftPWM(angleOut); 
        setRightPWM(-angleOut); 

        // 6. Stall Detection (Check every 0.1s)
        if (now > sampleTime + 100000) {
           if (abs(encRight.read() - sampleRight) < 2 && abs(encLeft.read() - sampleLeft) < 2) { 
               // If stalled, stop motors and exit to prevent overheating
               setRightPWM(0); 
               setLeftPWM(0); 
               return; 
           }
           sampleTime = now;
           sampleRight = encRight.read();
           sampleLeft = encLeft.read();
        }

        // 7. Update State
        error_old = error; 
        t_old = now;

        // Optional: Debugging
        // Serial.printf("Err: %.2f | Out: %.2f\n", error, angleOut);
        
        delay(5); // Small delay to prevent CPU hogging and IMU jitter
    }
}