#include "pidstraight.h"
#include <pidrotate.h>
#include <sidedist.h>

// PID for distance
double Kp_dist = 0; //0.15
double Ki_dist = 0;
double Kd_dist = 0; //0.01

// PID for angle offset
double Kp_angle = 0.85; //1.8 //1
double Ki_angle = 0.18;  //0.02
double Kd_angle = 0.09; //0.3

// PID for Wall Following (Steering)
double kP_left = 0.125  ; // Tune this! 0.4
double kI_left = 0.0; // 0.5
double kD_left = 0.25; // 0.1

double kP_right = 0; // Tune this!
double kI_right = 0.0;
double kD_right = 0;

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

double TARGET_LEFT = -1.0;
double TARGET_RIGHT = -1.0;

double getDistError() {
    static double heldError = 0;   // remembers last good error

    double leftDist = getLeftSideDist();
    double rightDist = getRightSideDist();

    int WALL_THRESHOLD = 120; // 120 mm (12 cm)
    if(TARGET_LEFT == -1.0) {
        TARGET_LEFT = leftDist;
        TARGET_RIGHT = rightDist;
    }

    bool leftWall = (leftDist > 0 && leftDist < WALL_THRESHOLD);
    bool rightWall = (rightDist > 0 && rightDist < WALL_THRESHOLD);

    if (leftWall && rightWall) {
        // Normal centering: offset by empirical targets to ensure true zero when centered
        heldError = (rightDist - TARGET_RIGHT) - (leftDist - TARGET_LEFT);
    } 
    else if (leftWall) {
        // follow left wall
        heldError = TARGET_LEFT - leftDist;
    } 
    else if (rightWall) {
        // follow right wall
        heldError = rightDist - TARGET_RIGHT;
    } 
    else {
        // no walls detected
        // hold previous error so robot continues straight relative to the wall it just left
    }

    // Constrain to +/- 80mm (8cm) like your reference code
    heldError = constrain(heldError, -8.0, 8.0);

    return heldError;
}


/*
void pidForward(double distance) {
    int basespeed = 200;
    Serial.printf("Hello pidForward Wall Follow! Distance: %f\n", distance);
    
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); 
    double goal_distance = encoder_per_mm * distance; 
    double goal_difference = 8 * encoder_per_mm;
    goal_distance = goal_distance + goal_difference; 
    
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);
    double avgEncoder = 0;

    // Initialize Timing and Distance PID variables
    double t_old = micros();

    double error_dist_left = goal_distance;
    double error_int_dist_left = 0;
    double error_deriv_dist_left = 0;
    double error_dist_left_old = error_dist_left;

    double error_dist_right = goal_distance;
    double error_int_dist_right = 0;
    double error_deriv_dist_right = 0;
    double error_dist_right_old = error_dist_right;

    // --- NEW: Left and Right Wall PID Variables ---
    double error_left = 0, error_int_left = 0, error_deriv_left = 0, error_left_old = 0;
    double error_right = 0, error_int_right = 0, error_deriv_right = 0, error_right_old = 0;

    double distOutLeft = 0;
    double distOutRight = 0;

    // --- Wall Constants ---
    const double IDEAL_LEFT = 51.5;  // Empirical center for Left
    const double IDEAL_RIGHT = 50.0; // Empirical center for Right
    const double TOLERANCE = 5.0;    // Tightened to 5mm to ignore sensor noise
    const double MAX_WALL = 120.0;   // Max distance to consider a wall "present"

    // Acceleration variables
    double startFactor = 0.5;     
    double accelRate = 2;       
    double rampProgress = 0.0;    

    while (true) {
        // --- 1. READ ENCODERS ---
        double currentLeft = -encLeft.read();
        double currentRight = -encRight.read();
        avgEncoder = (abs(currentLeft) + abs(currentRight)) / 2.0;

        // --- 2. EXIT CONDITIONS ---
        if (avgEncoder >= goal_distance) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        if (abs(goal_distance - currentLeft) <= 20 && abs(goal_distance - currentRight) <= 20) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500); 
            return;
        }

        if (front() < 70) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        // --- 3. WALL FOLLOWING ERROR CALCULATION ---
        double left_wall = getLeftSideDist();
        double right_wall = getRightSideDist();
        
        bool has_left = (left_wall > 0 && left_wall < MAX_WALL);
        bool has_right = (right_wall > 0 && right_wall < MAX_WALL);

        error_left = 0;
        error_right = 0;

        // Calculate Left Error
        if (has_left) {
            double raw_err_left = IDEAL_LEFT - left_wall; // Positive if too close to left wall
            if (abs(raw_err_left) > TOLERANCE) {
                error_left = raw_err_left;
            } else {
                error_int_left = 0; // Prevent integral windup in deadband
            }
        }

        // Calculate Right Error
        if (has_right) {
            double raw_err_right = IDEAL_RIGHT - right_wall; // Positive if too close to right wall
            if (abs(raw_err_right) > TOLERANCE) {
                error_right = raw_err_right;
            } else {
                error_int_right = 0; // Prevent integral windup in deadband
            }
        }

        // --- 4. PID CALCULATIONS ---
        error_dist_left = goal_distance - currentLeft;
        error_dist_right = goal_distance - currentRight;
        
        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3; // Safety fallback

        // Distance PID integrals & derivatives
        error_int_dist_left += error_dist_left * dt;
        error_int_dist_right += error_dist_right * dt;
        error_deriv_dist_left = (error_dist_left - error_dist_left_old) / dt;
        error_deriv_dist_right = (error_dist_right - error_dist_right_old) / dt;

        // Left Wall PID integrals & derivatives
        error_int_left += error_left * dt;
        error_deriv_left = (error_left - error_left_old) / dt;
        
        // Right Wall PID integrals & derivatives
        error_int_right += error_right * dt;
        error_deriv_right = (error_right - error_right_old) / dt;

        // Calculate Outputs
        distOutLeft = Kp_dist * error_dist_left + Ki_dist * error_int_dist_left + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        
        double steerOutLeft = kP_left * error_left + kI_left * error_int_left + kD_left * error_deriv_left;
        double steerOutRight = kP_right * error_right + kI_right * error_int_right + kD_right * error_deriv_right;

        // Combine Steering. (steerOutLeft pushes right, steerOutRight pushes left)
        double steerOutTotal = steerOutLeft - steerOutRight;

        // Apply Steering to PWM
        double rightPWM = basespeed + (distOutRight - steerOutTotal) * 1.25;
        double leftPWM = basespeed + (distOutLeft + steerOutTotal) * 1.25;

        // --- 5. SMOOTH ACCELERATION RAMP ---
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        rightPWM *= rampFactor;
        leftPWM *= rampFactor;

        // Deadband PWM adjustments to prevent motor stalling
        if (rightPWM < 30 && (rightPWM >= 0 && error_dist_right > 0)) rightPWM = 50;
        if (leftPWM < 30 && (leftPWM >= 0 && error_dist_left > 0)) leftPWM = 50;
        if (rightPWM > -30 && (rightPWM <= 0 && error_dist_right < 0)) rightPWM = -50;
        if (leftPWM > -30 && (leftPWM <= 0 && error_dist_left < 0)) leftPWM = -50;

        rightPWM = constrain(rightPWM, -200, 200);
        leftPWM = constrain(leftPWM, -200, 200);

        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
        Serial.printf("L Wall: %.1f | R Wall: %.1f | L Err: %.1f | R Err: %.1f | Steer: %.2f\n", 
                      left_wall, right_wall, error_left, error_right, steerOutTotal);
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_left_old = error_left;
        error_right_old = error_right;
        t_old = t_now;
    }
}
*/

void pidForwardIMU(double remaining_dist){
    int basespeed = 100;
    Serial.printf("Hello pidForward! Distance: %f\n", remaining_dist);
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); //converts mm to encoder ticks
    double goal_distance = encoder_per_mm * remaining_dist; // target distance in encoder ticks
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
    double startFactor = 0.5;     // Start at 20% of output
    double accelRate = 2.0;       // Controls how fast it ramps
    double rampProgress = 0.0;    

    while (true) {

        // if (avgEncoder >= goal_distance) {
        //     setRightPWM(0);
        //     setLeftPWM(0);
        //     delay(500);
        //     return;
        // }
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


        // --- 4. Smooth acceleration ramp ---
        // rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);

        // // Exponential ease-in ramp curve
        // double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        // // Apply ramp factor to PWM outputs
        // rightPWM *= rampFactor;
        // leftPWM *= rampFactor;

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




void pidForward(double distance) {
    int basespeed = 200;
    Serial.printf("Hello pidForward Wall Follow! Distance: %f\n", distance);
    
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); 
    double goal_distance = encoder_per_mm * distance; 
    double goal_difference = 8 * encoder_per_mm;
    goal_distance = goal_distance + goal_difference; 
    
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);
    double avgEncoder = 0;

    // Initialize Timing and Distance PID variables
    double t_old = micros();

    double error_dist_left = goal_distance;
    double error_int_dist_left = 0;
    double error_deriv_dist_left = 0;
    double error_dist_left_old = error_dist_left;

    double error_dist_right = goal_distance;
    double error_int_dist_right = 0;
    double error_deriv_dist_right = 0;
    double error_dist_right_old = error_dist_right;

    // --- Left and Right Wall PID Variables ---
    double error_left = 0, error_int_left = 0, error_deriv_left = 0, error_left_old = 0;
    double error_right = 0, error_int_right = 0, error_deriv_right = 0, error_right_old = 0;

    double distOutLeft = 0;
    double distOutRight = 0;
    double steerOutTotal = 0;

    // --- Wall Constants ---
    const double IDEAL_LEFT = 53.5;  
    const double IDEAL_RIGHT = 50.0; 
    const double TOLERANCE = 5.0;    
    const double MAX_WALL = 120.0;   

    // Acceleration variables
    double startFactor = 0.5;     
    double accelRate = 2;       
    double rampProgress = 0.0;    

    while (true) {
        // --- 1. READ ENCODERS ---
        double currentLeft = -encLeft.read();
        double currentRight = -encRight.read();
        avgEncoder = (abs(currentLeft) + abs(currentRight)) / 2.0;

        // --- 2. EXIT CONDITIONS ---
        if (avgEncoder >= goal_distance) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        if (abs(goal_distance - currentLeft) <= 20 && abs(goal_distance - currentRight) <= 20) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500); 
            return;
        }

        if (front() < 70) {
            setRightPWM(0);
            setLeftPWM(0);
            delay(500);
            return;
        }

        // --- TIMING ---
        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3; 

        // --- 3. EXCLUSIVE WALL FOLLOWING LOGIC ---
        double left_wall = getLeftSideDist();
        double right_wall = getRightSideDist();
        
        bool has_left = (left_wall > 0 && left_wall < MAX_WALL);
        bool has_right = (right_wall > 0 && right_wall < MAX_WALL);

        error_left = 0;
        error_right = 0;
        steerOutTotal = 0;

        if (has_left) {
            // PRIORITY 1: Use Left Wall Only
            double raw_err_left = IDEAL_LEFT - left_wall; 
            if (abs(raw_err_left) > TOLERANCE) {
                error_left = raw_err_left;
            } else {
                error_int_left = 0; 
            }

            error_int_left += error_left * dt;
            error_deriv_left = (error_left - error_left_old) / dt;
            
            steerOutTotal = kP_left * error_left + kI_left * error_int_left + kD_left * error_deriv_left;

            // Clear right wall memory to prevent integral windup while inactive
            error_int_right = 0;
            error_right_old = 0;

        } 
        else if (has_right) {
            // PRIORITY 2: Use Right Wall Only (If no left wall)
            double raw_err_right = IDEAL_RIGHT - right_wall; 
            if (abs(raw_err_right) > TOLERANCE) {
                error_right = raw_err_right;
            } else {
                error_int_right = 0; 
            }

            error_int_right += error_right * dt;
            error_deriv_right = (error_right - error_right_old) / dt;
            
            // Negative sign: positive right error pushes the mouse left
            steerOutTotal = -(kP_right * error_right + kI_right * error_int_right + kD_right * error_deriv_right);

            // Clear left wall memory 
            error_int_left = 0;
            error_left_old = 0;

        } 
        else {
            // PRIORITY 3: No Walls Detected - Handoff to IMU
            double current_dist_mm = avgEncoder / encoder_per_mm;
            double remaining_dist = distance - current_dist_mm;
            
            Serial.printf("Lost walls! Handing off to IMU for remaining %.1f mm\n", remaining_dist);
            
            if (remaining_dist > 0) {
                pidForwardIMU(remaining_dist); 
            }
            return; // Exit this function entirely; the IMU function handles the rest
        }


        // --- 4. DISTANCE PID CALCULATIONS ---
        error_dist_left = goal_distance - currentLeft;
        error_dist_right = goal_distance - currentRight;
        
        error_int_dist_left += error_dist_left * dt;
        error_int_dist_right += error_dist_right * dt;
        error_deriv_dist_left = (error_dist_left - error_dist_left_old) / dt;
        error_deriv_dist_right = (error_dist_right - error_dist_right_old) / dt;

        distOutLeft = Kp_dist * error_dist_left + Ki_dist * error_int_dist_left + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        
        // Apply Steering to PWM
        double rightPWM = basespeed + (distOutRight - steerOutTotal) * 1.25;
        double leftPWM = basespeed + (distOutLeft + steerOutTotal) * 1.25;

        // --- 5. SMOOTH ACCELERATION RAMP ---
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        rightPWM *= rampFactor;
        leftPWM *= rampFactor;

        // Deadband PWM adjustments to prevent motor stalling
        if (rightPWM < 30 && (rightPWM >= 0 && error_dist_right > 0)) rightPWM = 50;
        if (leftPWM < 30 && (leftPWM >= 0 && error_dist_left > 0)) leftPWM = 50;
        if (rightPWM > -30 && (rightPWM <= 0 && error_dist_right < 0)) rightPWM = -50;
        if (leftPWM > -30 && (leftPWM <= 0 && error_dist_left < 0)) leftPWM = -50;

        rightPWM = constrain(rightPWM, -200, 200);
        leftPWM = constrain(leftPWM, -200, 200);

        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
        Serial.printf("L Wall: %.1f | R Wall: %.1f | Steer: %.2f\n", left_wall, right_wall, steerOutTotal);
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_left_old = error_left;
        error_right_old = error_right;
        t_old = t_now;
    }
}



/*
void pidForward(double distance) {
    int basespeed = 200;
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
    double startFactor = 0.5;     // Start at 20% of output
    double accelRate = 6;       // Controls how fast it ramps
    double rampProgress = 0.0;    // Goes from 0 → 1 across distance;    

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

        rightPWM = constrain(rightPWM, -200, 200);
        leftPWM = constrain(leftPWM, -200, 200);

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
*/

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
