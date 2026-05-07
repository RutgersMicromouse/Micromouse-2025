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
double kP_left = 0.1; // Tune this! 0.4
double kI_left = 0.125; // 0.5
double kD_left = 0.09; // 0.1

double kP_right = 0.11; // Tune this!
double kI_right = 0.125;
double kD_right = 0.09;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};
double leftDist = 0;
double rightDist = 0;

void pidForwardSetup(){
    leftDist = getLeftSideDist();
    rightDist = getRightSideDist();
}

double TARGET_LEFT = 696969;
double TARGET_RIGHT = -1.0;

double getDistError() {
    static double heldError = 0;   // remembers last good error

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



void pidForwardIMU(double remaining_dist){
    int basespeed = 200;
    Serial.printf("Hello pidForward! Distance: %f\n", remaining_dist);
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); //converts mm to encoder ticks
    double goal_distance = encoder_per_mm * remaining_dist; // target distance in encoder ticks
    double goal_difference = -1*encoder_per_mm;
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

    // if (abs(goal_angle - angle()) > 2) {
    //     turnTo(goal_angle);
    // }

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
    double accelRate = 1.0;       // Controls how fast it ramps
    double rampProgress = 0.0;    

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


        // Stop if too close to wall
        if (front() < 70) {
            if (error_angle > 10) {
                turnTo(goal_angle); // Turn right if angle is positive
                break;
            } else if (error_angle < -10) {
                turnTo(goal_angle); // Turn left if angle is negative
                break;
            }

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

        rightPWM = constrain(rightPWM, -150, 150);
        leftPWM = constrain(leftPWM, -150, 150);

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
    //Serial.printf("Hello pidForward Wall Follow! Distance: %f\n", distance);
    
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI); 
    double goal_distance = encoder_per_mm * distance; 
    double goal_difference = -10 * encoder_per_mm;
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
    const double MAX_WALL = 130.0;   

    // Acceleration variables
    double startFactor = 0.5;     
    double accelRate = 1;       
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
            //Serial.printf("Left wall detected: %.1f mm\n", left_wall);
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
            //Serial.printf("Right wall detected: %.1f mm\n", right_wall);
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
            
            //Serial.printf("Lost walls! Handing off to IMU for remaining %.1f mm\n", remaining_dist);
            
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

        rightPWM = constrain(rightPWM, -150, 150);
        leftPWM = constrain(leftPWM, -150, 150);

        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
        //Serial.printf("L Wall: %.1f | R Wall: %.1f | Steer: %.2f\n", left_wall, right_wall, steerOutTotal);
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_left_old = error_left;
        error_right_old = error_right;
        t_old = t_now;
    }
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
