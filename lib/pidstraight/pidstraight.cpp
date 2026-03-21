#include "pidstraight.h"
#include <pidrotate.h>
#include <sidedist.h>

// --- PID for distance (Forward Progress) ---
double Kp_dist = 0.0; 
double Ki_dist = 0.0;
double Kd_dist = 0.0; 

// --- PID for Angle Anchor (The core steering you requested) ---
double Kp_angle = 0.8; 
double Ki_angle = 0.0; 
double Kd_angle = 0.1;//0.2; //.2

// --- Unified Navigation Constants (The "Nudge") ---
double Km = 0.4;//0.2;//0.7;  // How much the walls shift the IMU target
double Td = 0;//0.1;//0.4;  // Damping for the wall shift

// --- Calibration and Thresholds ---
double TARGET_LEFT = 0;  
double TARGET_RIGHT = 0; 
const double MAX_WALL = 130.0; 
const double I_LIMIT = 50.0; 
double identity_diag[4] = {0.0, 90.0, 180.0, 270.0};

void pidForwardSetup() {
    TARGET_LEFT = getLeftSideDist();
    TARGET_RIGHT = getRightSideDist();
}

void pidForward(double distance_mm) {
    int n = 0;
    int basespeed = 135;
    double encoder_per_mm = TICKS_PER_ROTATION / (WHEEL_DIAM * PI);
    double difference = 3 * encoder_per_mm; // how many ticks off we want to be at the end, positive means we want to undershoot
    double goal_ticks = (distance_mm * encoder_per_mm)+difference;

    encLeft.write(0);
    encRight.write(0);

    double t_old = micros(); 
    double err_steer_prev = 0; 
    double err_steer_int = 0;
    double err_wall_prev = 0;

    double last_l_wall = TARGET_LEFT;
    double last_r_wall = TARGET_RIGHT;

    // --- 1. Find Absolute Goal Angle ---
    double goal_angle = 0;
    double curr_ang = angle();
    double min_diff = 1000;
    for(int i = 0; i < 4; i++) {
        double diff = identity_diag[i] - curr_ang;
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        if (abs(diff) < min_diff) {
            min_diff = abs(diff);
            goal_angle = identity_diag[i];
        }
    }

    // if (abs(goal_angle - angle()) > 30 && goal_angle != 0) {
    //     turnTo(goal_angle);
    // } else if (goal_angle ==0){
    //     if (angle() > 30 || angle() > 330){
    //         turnTo(0);
    //     }
    // }

    while (true) {
        // --- 2. SENSING ---
        double avgTicks = (abs(-encLeft.read()) + abs(-encRight.read())) / 2.0;
        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 0.001;
        t_old = t_now;

        if (avgTicks >= goal_ticks || front() < 70) break;

        // --- 3. WALL SENSING (The "Nudge") ---
        double l_wall = getLeftSideDist();
        double r_wall = getRightSideDist();
        if (l_wall == -1) l_wall = last_l_wall;
        if (r_wall == -1) r_wall = last_r_wall;
        last_l_wall = l_wall;
        last_r_wall = r_wall;

        double wallError = 0;
        bool hasL = (l_wall > 0 && l_wall < MAX_WALL);
        bool hasR = (r_wall > 0 && r_wall < MAX_WALL);

        if (hasL && hasR) {
            wallError = (l_wall - TARGET_LEFT) - (r_wall - TARGET_RIGHT);
        } else if (hasL) {
            wallError = (l_wall - TARGET_LEFT) * 2.0;
        } else if (hasR) {
            wallError = -(r_wall - TARGET_RIGHT) * 2.0;
        }

        // Calculate wall correction to shift the IMU target
        double wallDeriv = (wallError - err_wall_prev) / dt;
        double wallNudge = (Km * wallError) + (Td * Km * wallDeriv);
        err_wall_prev = wallError;

        //Serial.printf("Wall Error: %.2f | Wall Nudge: %.2f | Goal Angle: %.2f\n", wallError, wallNudge, goal_angle);
        // --- 4. CORE IMU PID (The Logic You Requested) ---
        // We add wallNudge to the angle error to "trick" the IMU into centering
        double current_err = (goal_angle - angle());
        while (current_err > 180) current_err -= 360;
        while (current_err < -180) current_err += 360;
        
        // Add the wall nudge here
        current_err -= wallNudge;

        err_steer_int = constrain(err_steer_int + (current_err * dt), -I_LIMIT, I_LIMIT);
        double d_term = (current_err - err_steer_prev) / dt;
        double steerOut = (Kp_angle * current_err) + (Ki_angle * err_steer_int) + (Kd_angle * d_term);
        //Serial.printf("Angle Error: %.2f | Steer Out: %.2f\n", current_err, steerOut);

        // --- 5. MOTOR MAPPING ---
        double leftPWM = basespeed + (steerOut * 1);
        double rightPWM = basespeed - (steerOut * 1);

        // Deadband & Kickstart
        if (abs(leftPWM) < 35) leftPWM = (leftPWM > 0) ? 40 : -40;
        if (abs(rightPWM) < 35) rightPWM = (rightPWM > 0) ? 40 : -40;
        if (n < 2) {
             leftPWM -= 25; 
             leftPWM *= 0.75;
             rightPWM *= 0.75;
             n++;
            }
        
        if(l_wall < 5) {
            leftPWM += 7;
            rightPWM -= 7;
        }

        if(r_wall < 5) {
            leftPWM -= 7;
            rightPWM += 7;
        }
        setLeftPWM(constrain(leftPWM, -180, 180));
        setRightPWM(constrain(rightPWM, -180, 180));
        
        err_steer_prev = current_err;
    }

    setLeftPWM(0);
    setRightPWM(0);
    delay(400);
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