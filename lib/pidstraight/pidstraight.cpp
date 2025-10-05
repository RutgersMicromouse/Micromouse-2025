#include "pidstraight.h"

// PID for distance (not used anymore in continuous mode)
double Kp_dist = 0.30;
double Ki_dist = 0;
double Kd_dist = 0;

// PID for angle offset
double Kp_angle = 2.5;
double Ki_angle = 0;
double Kd_angle = 0;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

// =====================================================
// Continuous Straight Driving (replaces old pidForward)
// =====================================================
void pidForward(double distance) {
    Serial.println("Hello pidForward (continuous)!");

    // Lock in the current angle as the goal
    double goal_angle = angle();

    // PID state
    double t_old = micros();
    double error_int_angle = 0;
    double error_angle_old = 0;

    while (true) {
        double t_now = micros();
        double dt = (t_now - t_old) / 1e6; // seconds
        if (dt <= 0) dt = 1e-3;

        // --- Angle error ---
        double error_angle = goal_angle - angle();

        // Wrap to [-180, 180]
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        // --- PID terms ---
        error_int_angle += error_angle * dt;
        double error_deriv_angle = (error_angle - error_angle_old) / dt;

        double angleOut = Kp_angle * error_angle
                        + Ki_angle * error_int_angle
                        + Kd_angle * error_deriv_angle;
        // --- Motor output ---
        int basePWM = 200;  // forward speed
        int leftPWM  = basePWM + angleOut;
        int rightPWM = basePWM - angleOut;

        


        if (angleOut>(255-basePWM)){
            //Spread Values
            double yMax = 700;
            double xMax = 255;
            // k = xMax/yMax^2
            double k = xMax/(yMax*yMax);  
            //function for spreading: k*input*abs(input)
            leftPWM = k*(leftPWM)*abs(leftPWM);
            rightPWM = k*(rightPWM)*abs(rightPWM);
        }
        

        // Clamp values
        leftPWM  = constrain(leftPWM,  -255, 255);
        rightPWM = constrain(rightPWM, -255, 255);

        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);

        // Update state
        error_angle_old = error_angle;
        t_old = t_now;

        if(front() < 90) { setRightPWM(0); setLeftPWM(0); return; }
    }
}

// =====================================================
// Left Wall Following (unchanged from your code)
// =====================================================
void pidForwardLeftWallFollow() {
    Serial.println("Hello pidForwardLeftWallFollow!");

    // Find the closest world angle axis
    double goal_angle;
    int closest_index = 0; // index of the closest world angle 
    double arr_diag[8];    // distances from current angle to world angles
    
    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - angle();

        // angle wrapping [-180, 180]
        if (arr_diag[i] > 180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        
        // if lowest angle difference, store index
        if(abs(arr_diag[i]) < abs(arr_diag[closest_index])) {
            closest_index = i;
        }  
    }    
    goal_angle = identity_diag[closest_index];

    Serial.print("Goal angle: ");
    Serial.println(goal_angle);

    // All of the variables we need for PID
    double t_old = micros();
    
    double error_angle = goal_angle - angle();
    // angle wrapping
    if (error_angle > 180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;
    
    double error_int_angle; 
    double error_deriv_angle;
    double error_angle_old = error_angle;
    
    double angleOut;

    // sampling to check for motor stalling
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();
    
    while (true) {
        // Guard Clauses:
        // 1. Take all left walls
        if (!leftWall()) { delay(120); setRightPWM(0); setLeftPWM(0); return; }
        
        // 2. Stall Condition, 0.1 second
        if (micros() > sampleTime + 1e5){
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { 
                setRightPWM(0); setLeftPWM(0); return; 
            }
            
            // Update stall sampler
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
        }

        // 3. Too close to the front wall
        if(front() < 90) { setRightPWM(0); setLeftPWM(0); return; }

        // P error
        error_angle = goal_angle - angle();
        // angle wrapping
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;
        
        // I error
        error_int_angle += error_angle * (micros() - t_old);
        
        // D error
        error_deriv_angle = (error_angle - error_angle_old)/(micros() - t_old);

        angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
        setLeftPWM(200 + angleOut); 
        setRightPWM(200 - angleOut); 

        // update error_angle_old, and t_old
        error_angle_old = error_angle; 
        t_old = micros();
    }
}
