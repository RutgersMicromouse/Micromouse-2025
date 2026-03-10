#include "pidstraight.h"

// PID for distance
double Kp_dist = 1.0;
double Ki_dist = 0.0;
double Kd_dist = 0.0; // Added a tiny bit of D to dampen stops

// PID for angle offset
double Kp_angle = 3.0;
double Ki_angle = 1.0;
double Kd_angle = 0.0;

double identity_diag[8] = {0.0, 45, 90, 135, 180, 225, 270, 315};

void pidForward(double distance) {
    Serial.println("--- Starting pidForward ---");
    
    // 1. Convert mm to encoder ticks
    // Ensure WHEEL_DIAM and TICKS_PER_ROTATION are defined in your header
    double goal_ticks = (distance * TICKS_PER_ROTATION) / (WHEEL_DIAM * PI);
    goal_ticks *= 1.65; // Calibration factor

    encLeft.write(0); 
    encRight.write(0);

    // 2. Find the closest world angle axis (Snap to 45-degree increments)
    double current_start_angle = angle();
    double goal_angle = identity_diag[0];
    double min_diff = 360.0;

    for (int i = 0; i < 8; i++) {
        double diff = identity_diag[i] - current_start_angle;
        // Angle wrapping
        while (diff > 180)  diff -= 360;
        while (diff < -180) diff += 360;
        
        if (abs(diff) < abs(min_diff)) {
            min_diff = diff;
            goal_angle = identity_diag[i];
        }
    }
    Serial.print("Target Angle: "); Serial.println(goal_angle);

    // PID State Variables
    double t_old = micros() / 1.0e6; // Work in seconds
    double error_dist_left_old = goal_ticks;
    double error_dist_right_old = goal_ticks;
    double error_angle_old = 0;
    
    double i_dist_l = 0, i_dist_r = 0, i_ang = 0;

    // Stall Detection Variables
    unsigned long sampleTime = millis();
    long lastLeftTicks = 0;
    long lastRightTicks = 0;

    while (true) {
        // --- 3. TIMING ---
        double t_now = micros() / 1.0e6;
        double dt = t_now - t_old;
        if (dt <= 0) dt = 0.001; // Avoid division by zero

        long currL = encLeft.read();
        long currR = encRight.read();

        // --- 4. GUARD CLAUSES ---
        // A. Distance Reached
        double errL = goal_ticks - currL;
        double errR = goal_ticks - currR;
        if (abs(errL) < 10 && abs(errR) < 10) break;

        // B. Stall Protection (Check every 100ms)
        if (millis() - sampleTime > 100) {
            if (abs(currL - lastLeftTicks) < 2 && abs(currR - lastRightTicks) < 2) {
                // Only trigger stall if we aren't already at the goal
                if (abs(errL) > 50) {
                    Serial.println("STALL DETECTED - Stopping.");
                    break;
                }
            }
            sampleTime = millis();
            lastLeftTicks = currL; lastRightTicks = currR;
        }

        // C. Obstacle Detection
        if (front() < 50) {
            Serial.println("FRONT WALL - Stopping.");
            break;
        }

        // --- 5. PID CALCULATIONS ---
        // Angle Error with Wrapping
        double errAng = goal_angle - angle();
        while (errAng > 180)  errAng -= 360;
        while (errAng < -180) errAng += 360;

        // Integrals
        i_dist_l += errL * dt;
        i_dist_r += errR * dt;
        i_ang    += errAng * dt;

        // Derivatives
        double d_dist_l = (errL - error_dist_left_old) / dt;
        double d_dist_r = (errR - error_dist_right_old) / dt;
        double d_ang    = (errAng - error_angle_old) / dt;

        // Output Summation
        double outL = (Kp_dist * errL) + (Ki_dist * i_dist_l) + (Kd_dist * d_dist_l);
        double outR = (Kp_dist * errR) + (Ki_dist * i_dist_r) + (Kd_dist * d_dist_r);
        double outA = (Kp_angle * errAng) + (Ki_angle * i_ang) + (Kd_angle * d_ang);

        // --- 6. MOTOR COMPENSATION (The 3:1 Ratio) ---
        // baseLeft + angle correction
        double leftPWM  = (outL + outA) * 1; 
        // baseRight - angle correction
        double rightPWM = (outR - outA);

        // --- 7. RATIO-PRESERVING CONSTRAINT ---
        // This ensures the 3:1 ratio is kept even if one motor hits max power (255)
        double maxRequested = max(abs(leftPWM), abs(rightPWM));
        double limit = 150.0; // Stay slightly below 255 for stability
        
        if (maxRequested > limit) {
            double scale = limit / maxRequested;
            leftPWM  *= scale;
            rightPWM *= scale;
        }

        // Deadband compensation: if power is too low to move, give it a tiny kick
        if (abs(leftPWM) > 0 && abs(leftPWM) < 35) leftPWM = (leftPWM > 0) ? 40 : -40;
        if (abs(rightPWM) > 0 && abs(rightPWM) < 35) rightPWM = (rightPWM > 0) ? 40 : -40;

        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);

        // Update History
        error_dist_left_old = errL;
        error_dist_right_old = errR;
        error_angle_old = errAng;
        t_old = t_now;
    }

    // 8. Final Stop
    setLeftPWM(0);
    setRightPWM(0);
    Serial.println("--- pidForward Finished ---");
}


void pidForwardLeftWallFollow() {
    Serial.println("Hello pidForwardLeftWallFollow!");

    // Find the closest world angle axis
    double goal_angle;
    int closest_index = 0; // index of the closest world angle 
    double arr_diag[8]; // distances from current angle to world angles
    
    for (int i = 0; i <= 7; i++) {

        arr_diag[i] = identity_diag[i] - angle();

        // angle wrapping
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
    
    double error_int_angle; double error_deriv_angle;
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
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { setRightPWM(0); setLeftPWM(0); return; }
            
            // Update stall sampler
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
        }

        // 3. Too close to the front wall
        if(front() < 90)          { setRightPWM(0); setLeftPWM(0); return; }

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
        setLeftPWM(200 + angleOut); setRightPWM(200 - angleOut); 


        // update error_angle_old, and t_old
        error_angle_old = error_angle; t_old = micros();
    }
}

