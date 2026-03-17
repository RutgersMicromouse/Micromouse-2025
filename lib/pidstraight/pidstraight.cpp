#include "pidstraight.h"
#include <pidrotate.h>

// PID for distance
double Kp_dist = 0; //0.15
double Ki_dist = 0;
double Kd_dist = 0; //0.01

// PID for angle offset
double Kp_angle = 0.85; //1.8 //1
double Ki_angle = 0.18;  //0.02
double Kd_angle = 0.09; //0.3

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

// Distance forward in mm
/*
void pidForward(double distance) {
    Serial.printf("Hello pidForward! Distance: %f\n", distance);
    double goal_distance = (TICKS_PER_ROTATION * distance / (WHEEL_DIAM * PI))/2.8; // Converts mm -> encoder ticks
        
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);
    double avgEncoder = 0;

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

        
    // Initialize PID variables
    double t_old = micros();

    double error_dist_left = goal_distance;
    double error_int_dist_left = 0;
    double error_deriv_dist_left = 0;
    double error_dist_left_old = error_dist_left;

    double error_dist_right = goal_distance;
    double error_int_dist_right = 0;
    double error_deriv_dist_right = 0;
    double error_dist_right_old = error_dist_right;

    double error_angle = goal_angle - angle();
    if (error_angle > 180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;

    double error_int_angle = 0;
    double error_deriv_angle = 0;
    double error_angle_old = error_angle;

    double distOutLeft = 0;
    double distOutRight = 0;
    double angleOut = 0;

    // Stall detection setup
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    // ✅ NEW: Reset acceleration parameters for each call
    double startFactor = 0.2;     // Start at 50% of output
    double accelRate = 1;       // Controls how fast it ramps (higher = faster)
    double rampProgress = 0.0;    // Goes from 0 → 1 across distance

    while (true) {
        // if (avgEncoder >= goal_distance) {
        //     setRightPWM(0);
        //     setLeftPWM(0);
        //     delay(500);
        //     return;
        // }

        // --- Guard Clauses ---
        if (abs(error_dist_left) <= 10 && abs(error_dist_right) <= 10) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        // Stall detection (0.1s)
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

        // Stop if too close to wall
        if (front() < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(2000);
            return;
        }

        // --- PID error calculations ---
        error_dist_left = goal_distance - encLeft.read();
        error_dist_right = goal_distance - encRight.read();
        error_angle = goal_angle - angle();
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3; // Safety fallback

        error_int_dist_left += error_dist_left * dt;
        error_int_dist_right += error_dist_right * dt;
        error_int_angle += error_angle * dt;

        error_deriv_dist_left = (error_dist_left - error_dist_left_old) / dt;
        error_deriv_dist_right = (error_dist_right - error_dist_right_old) / dt;
        error_deriv_angle = (error_angle - error_angle_old) / dt;

        distOutLeft = Kp_dist * error_dist_left + Ki_dist * error_int_dist_left + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;

        double rightPWM = (distOutRight - angleOut)*1.25;
        double leftPWM = (distOutLeft + angleOut)*1.25;

        // --- ✅ NEW: Smooth acceleration ramp ---
        avgEncoder = (abs(encLeft.read()) + abs(encRight.read())) / 2.0;
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);

        double leftWheel = encLeft.read();
        double rightWheel = encRight.read();
        double wheelDiff = leftWheel - rightWheel;   //positive if left is ahead, negative if right is ahead

        // Exponential ease-in ramp curve
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        // Apply ramp factor to PWM outputs
        rightPWM *= rampFactor;
        leftPWM *= rampFactor;

        // Limit PWM range
        rightPWM = constrain(rightPWM, -200, 200);
        leftPWM = constrain(leftPWM, -200, 200);

        // Apply PWM
        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
        Serial.printf("Goal Dist: %6.2f | Curr Dist: %6.2f | Goal Ang: %6.2f | Curr Ang: %6.2f | L PWM: %6.2f | R PWM: %6.2f | Angle Error: %6.2f\n", 
                      goal_distance, 
                      avgEncoder, 
                      goal_angle, 
                      angle(), 
                      leftPWM, 
                      rightPWM,
                      error_angle
                      );
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_angle_old = error_angle;
        t_old = t_now;
    }
    Serial.println("Exited PID loop");
}
*/


void pidForward(double distance) {
    int basespeed = 800;
    Serial.printf("Hello pidForward! Distance: %f\n", distance);
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); //converts mm to encoder ticks
    double goal_distance = encoder_per_mm * distance; // target distance in encoder ticks
    double goal_difference = 8*encoder_per_mm;
    goal_distance = goal_distance + goal_difference; // empirical adjustment to hit the target more accurately (tune this!)
    
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);
    double avgEncoder = 0;

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

    if (abs(goal_angle - angle()) > 2) {
        turnTo(goal_angle);
    }

    // Initialize PID variables
    double t_old = micros();

    double error_dist_left = goal_distance;
    double error_int_dist_left = 0;
    double error_deriv_dist_left = 0;
    double error_dist_left_old = error_dist_left;

    double error_dist_right = goal_distance;
    double error_int_dist_right = 0;
    double error_deriv_dist_right = 0;
    double error_dist_right_old = error_dist_right;

    double error_angle = goal_angle - angle();
    if (error_angle > 180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;

    double error_int_angle = 0;
    double error_deriv_angle = 0;
    double error_angle_old = error_angle;

    double distOutLeft = 0;
    double distOutRight = 0;
    double angleOut = 0;

    // --- Stall detection setup (Fixed Polarity) ---
    double sampleTime = micros();
    double sampleRight = encRight.read(); // Inverted polarity (not?)
    double sampleLeft = encLeft.read();   // Inverted polarity (not?)

    // Acceleration variables
    double startFactor = 0.15;     // Start at 20% of output
    double accelRate = 0.8;       // Controls how fast it ramps
    double rampProgress = 0.5   ;    

    while (true) {

        if (avgEncoder >= goal_distance) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        // --- 1. READ AND INVERT ENCODERS ---
        // Multiply by -1 to fix the backward counting!
        double currentLeft = -encLeft.read();
        double currentRight = -encRight.read();

        // Calculate absolute distance for the ramp
        avgEncoder = (abs(currentLeft) + abs(currentRight)) / 2.0;

        // --- 2. Guard Clauses ---
        // Widen the window to 10 ticks so it doesn't skip it at high speeds
        if (abs(goal_distance - currentLeft) <= 20 && abs(goal_distance - currentRight) <= 20) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500); // Small delay to see it stop
            return;
        }

        //  // TEMPORARILY COMMENT OUT STALL DETECTION WHILE TUNING
        // if (micros() > sampleTime + 1e5) {
        //     if (abs(currentRight - sampleRight) < 2 || abs(currentLeft - sampleLeft) < 2) {
        //         setRightPWM(0);
        //         setLeftPWM(0);
        //         return;
        //     }
        //     sampleTime = micros();
        //     sampleRight = currentRight;
        //     sampleLeft = currentLeft;
        // }
        

        // Stop if too close to wall
        if (front() < 70) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        // --- 3. PID error calculations ---
        error_dist_left = goal_distance - currentLeft;
        error_dist_right = goal_distance - currentRight;
        
        error_angle = goal_angle - angle();
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3; // Safety fallback

        error_int_dist_left += error_dist_left * dt;
        error_int_dist_right += error_dist_right * dt;
        error_int_angle += error_angle * dt;

        error_deriv_dist_left = (error_dist_left - error_dist_left_old) / dt;
        error_deriv_dist_right = (error_dist_right - error_dist_right_old) / dt;
        error_deriv_angle = (error_angle - error_angle_old) / dt;

        distOutLeft = Kp_dist * error_dist_left + Ki_dist * error_int_dist_left + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;

        double rightPWM = basespeed + (distOutRight - angleOut) * 1.25;
        double leftPWM = basespeed + (distOutLeft + angleOut) * 1.25;


        //--- 4. Smooth acceleration ramp ---
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);

        // Exponential ease-in ramp curve
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        // Apply ramp factor to PWM outputs
        rightPWM *= rampFactor;
        leftPWM *= rampFactor;

        // Limit PWM range
        // if (error_dist_left> 0.6*goal_distance && error_dist_right > 0.6*goal_distance) { // If we're far, allow higher speeds
        //     rightPWM = constrain(rightPWM, 120, 400);
        //     leftPWM = constrain(leftPWM, 120, 400);
        // } else if (abs(error_dist_left)  < 5 && abs(error_dist_right) < 5) { // If we're close, be more gentle
        //     if (error_dist_left > 0 && error_dist_right > 0) { // If we're close but still have a lot of distance left, allow more power
        //         rightPWM = constrain(rightPWM, 70, 400);
        //         leftPWM = constrain(leftPWM, 70, 400);
        //     } else { // If we're close and within range, be gentle
        //         rightPWM = constrain(rightPWM, -400, -70);
        //         leftPWM = constrain(leftPWM, -400, -70);
        //     }
        //     rightPWM = constrain(rightPWM, -400, 400);
        //     leftPWM = constrain(leftPWM, -400, 400);
        // } else {
        //     rightPWM = constrain(rightPWM, -400, 400);
        //     leftPWM = constrain(leftPWM, -400, 400);
        // }

        // Apply PWM
        if (rightPWM < 30 && (rightPWM >= 0 && error_dist_right > 0)) {
            rightPWM = 50;
        }
        if (leftPWM < 30 && (leftPWM >= 0 && error_dist_left > 0)){
            leftPWM = 50;
        }
        if (rightPWM > -30 && (rightPWM <= 0 && error_dist_right < 0)){
            rightPWM = -50;
        }
        if (leftPWM > -30 && (leftPWM <= 0 && error_dist_left < 0)) {
            leftPWM = -50;
        }
        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
        Serial.printf("Goal Dist: %6.2f\t| Curr L: %6.2f\t| Error L: %6.2f\t| L PWM: %6.2f \t| R PWM: %6.2f\n", 
                      goal_distance, 
                      currentLeft, 
                      error_dist_left, 
                      leftPWM, 
                      rightPWM
                      );
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_angle_old = error_angle;
        t_old = t_now;
    }
    Serial.println("Exited PID loop");
}


/*
void pidForward(double distance) {
    Serial.printf("Hello pidForward! Distance: %f\n", distance);
    double goal_distance = (TICKS_PER_ROTATION * distance / (WHEEL_DIAM * PI)); // Converts mm -> encoder ticks

    // Reset encoders
    encLeft.write(0);
    encRight.write(0);

    // --- 1. Angle Setup: Find the closest world axis to snap to ---
    double goal_angle;
    int closest_index = 0;
    double arr_diag[8];
    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - angle();
        // Handle angle wrap-around
        if (arr_diag[i] > 180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) {
            closest_index = i;
        }
    }
    goal_angle = identity_diag[closest_index];

    if (goal_angle-angle() > 20 || goal_angle-angle() < -20) {
        turnTo(goal_angle);
    }

    // --- 2. Initialize Angle PID Variables ---
    double t_old = micros();
    double error_int_angle = 0;
    
    // Pre-calculate initial error to prevent a massive derivative spike on loop 1
    double error_angle_old = goal_angle - angle();
    if (error_angle_old > 180) error_angle_old -= 360;
    if (error_angle_old < -180) error_angle_old += 360;

    while (1){
        // --- 3. Exit Conditions ---
        if(-encLeft.read() >= goal_distance || -encRight.read() >= goal_distance) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }
        if (front() < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(2000);
            return;
        }  

        // --- 4. Angle PID Calculations ---
        double current_angle = angle();
        double error_angle = goal_angle - current_angle;
        
        // Handle angle wrap-around (-180 to 180)
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3; // Safety fallback

        error_int_angle += error_angle * dt;
        double error_deriv_angle = (error_angle - error_angle_old) / dt;

        // Calculate total angle correction
        double angleOut = (Kp_angle * error_angle) + (Ki_angle * error_int_angle) + (Kd_angle * error_deriv_angle);

        // --- 5. Apply Base Speed + Angle Correction ---
        double base_speed = 125.0; // Your chosen constant speed
        
        // Subtract from right, add to left (adjust signs if it steers the wrong way)
        double rightPWM = base_speed - angleOut; 
        double leftPWM = base_speed + angleOut;

        // Safety cap so the motors don't receive invalid PWM signals
        rightPWM = constrain(rightPWM, -400, 400);
        leftPWM = constrain(leftPWM, -400, 400);

        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug Print
        Serial.printf("Goal: %.1f | Front: %d |  L Enc: %d | R Enc: %d | Ang Err: %.2f | L PWM: %.0f | R PWM: %.0f\n", 
            goal_distance, front(), -encLeft.read(), -encRight.read(), error_angle, leftPWM, rightPWM);

        // Update tracking variables for the next loop
        error_angle_old = error_angle;
        t_old = t_now;
    }
    
    // Safety stop (though the returns above should catch it)
    setRightPWM(0);
    setLeftPWM(0);
}

*/

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
