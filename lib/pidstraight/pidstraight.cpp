#include "pidstraight.h"

// PID for distance
double Kp_dist = 1.0;
double Ki_dist = 0.0;
double Kd_dist = 0.0; // Added a tiny bit of D to dampen stops

// PID for angle offset
double Kp_angle = 0.0;
double Ki_angle = 0.0;
double Kd_angle = 0.0;

double identity_diag[8] = {0.0, 45, 90, 135, 180, 225, 270, 315};


void pidForward(double distance) {
    Serial.println("--- Starting pidForward ---");

    // --- CONFIGURATION ---
    double baseSpeed  = 120.0; // The cruising speed
    double speedLimit = 200.0; // The absolute maximum PWM allowed
    // ---------------------
    
    // 1. Convert mm to encoder ticks
    double goal_ticks = (distance * TICKS_PER_ROTATION) / (WHEEL_DIAM * PI);
    goal_ticks *= 1.65; // Your calibration factor

    encLeft.write(0); 
    encRight.write(0);

    // 2. Snap to the closest 45-degree world axis
    double current_start_angle = angle();
    double goal_angle = identity_diag[0];
    double min_diff = 360.0;

    for (int i = 0; i < 8; i++) {
        double diff = identity_diag[i] - current_start_angle;
        while (diff > 180)  diff -= 360;
        while (diff < -180) diff += 360;
        
        if (abs(diff) < abs(min_diff)) {
            min_diff = diff;
            goal_angle = identity_diag[i];
        }
    }

    // PID State Variables
    double t_old = micros() / 1.0e6;
    double error_angle_old = 0;
    double i_ang = 0;

    // Stall Detection Variables
    unsigned long sampleTime = millis();
    long lastLeftTicks = 0;
    long lastRightTicks = 0;

    Serial.print("current_start_angle: "); Serial.println(current_start_angle);
    Serial.print("goal_angle: "); Serial.println(goal_angle);
    Serial.print("Target Angle: "); Serial.println(goal_angle);

    while (true) {
        // --- 3. TIMING ---
        double t_now = micros() / 1.0e6;
        double dt = t_now - t_old;
        if (dt <= 0) dt = 0.001; 

        long currL = encLeft.read();
        long currR = encRight.read();

        // --- 4. GUARD CLAUSES ---
        double errL = goal_ticks - currL;
        double errR = goal_ticks - currR;

        // Stop if both wheels are within a small threshold of the goal
        if (abs(errL) < 15 && abs(errR) < 15) break;

        // Stall Protection (Check every 100ms)
        if (millis() - sampleTime > 100) {
            if (abs(currL - lastLeftTicks) < 2 && abs(currR - lastRightTicks) < 2) {
                if (abs(errL) > 50) { 
                    Serial.println("STALL DETECTED");
                    break;
                }
            }
            sampleTime = millis();
            lastLeftTicks = currL; lastRightTicks = currR;
        }

        // Obstacle Detection
        if (front() < 50) {
            Serial.println("FRONT WALL");
            break;
        }

        // --- 5. PID CALCULATIONS ---
        
        // Angle PID (The "Correction" factor)
        double errAng = goal_angle - angle();
        while (errAng > 180)  errAng -= 360;
        while (errAng < -180) errAng += 360;

        i_ang += errAng * dt;
        double d_ang = (errAng - error_angle_old) / dt;
        double outA  = (Kp_angle * errAng) + (Ki_angle * i_ang) + (Kd_angle * d_ang);

        // Distance Calculation (The "Drive" factor)
        // Kp_dist acts as the "braking" aggressiveness
        double driveL = errL * Kp_dist;
        double driveR = errR * Kp_dist;

        // --- 6. APPLY SPEED CONSTRAINTS ---
        
        // Step A: Determine the Drive Power (Capped at baseSpeed)
        double motorPowerL = constrain(driveL, -baseSpeed, baseSpeed);
        double motorPowerR = constrain(driveR, -baseSpeed, baseSpeed);

        // Step B: Add the Angle Correction
        double leftPWM  = motorPowerL + outA;
        double rightPWM = motorPowerR - outA;

        // Step C: Apply Absolute speedLimit (The Ceiling)
        leftPWM  = constrain(leftPWM, -speedLimit, speedLimit)*1.4;
        rightPWM = constrain(rightPWM, -speedLimit, speedLimit);

        // --- 7. DEADZONE KICK ---
        // Minimum power to overcome static friction if still far from goal
        if (abs(leftPWM) < 35 && abs(errL) > 15)  leftPWM = (leftPWM > 0) ? 40 : -40;
        if (abs(rightPWM) < 35 && abs(errR) > 15) rightPWM = (rightPWM > 0) ? 40 : -40;

        setLeftPWM(leftPWM);
        setRightPWM(rightPWM);


        Serial.print("Error Angle: "); Serial.println(errAng); //If error to the right, its negative, else positive.
        Serial.print("Left PWM: "); Serial.print(leftPWM);
        Serial.print("\t|\tRight PWM: "); Serial.println(rightPWM);
        Serial.println("");



        // Update History
        error_angle_old = errAng;
        t_old = t_now;
    }

    // 8. Hard Stop
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

