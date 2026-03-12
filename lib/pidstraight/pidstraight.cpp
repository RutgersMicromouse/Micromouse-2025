#include "pidstraight.h"

// PID for distance
double Kp_dist = 0.8;
double Ki_dist = 0;
double Kd_dist = 0;

// PID for angle offset
double Kp_angle = 3.63;
double Ki_angle = -0.2;
double Kd_angle = 0.01;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

// Distance forward in mm
void pidForward(double distance) {
    Serial.printf("Hello pidForward! Distance: %f\n", distance);
        double goal_distance = TICKS_PER_ROTATION * distance / (WHEEL_DIAM * PI); // Converts mm -> encoder ticks
        
    // Reset encoders
    encLeft.write(0);
    encRight.write(0);

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
    double startFactor = 0.1;     // Start at 50% of output
    double accelRate = 3;       // Controls how fast it ramps (higher = faster)
    double rampProgress = 0.0;    // Goes from 0 → 1 across distance

    while (true) {
        // --- Guard Clauses ---
        if (abs(error_dist_left) <= 3 && abs(error_dist_right) <= 3) {
            setRightPWM(0);
            setLeftPWM(0);
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
        double avgEncoder = (abs(encLeft.read()) + abs(encRight.read())) / 2.0;
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);

        // Exponential ease-in ramp curve
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        // Apply ramp factor to PWM outputs
        rightPWM *= rampFactor;
        leftPWM *= rampFactor;

        // Limit PWM range
        rightPWM = constrain(rightPWM, -400, 400);
        leftPWM = constrain(leftPWM, -400, 400);

        // Apply PWM
        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        // Debug print
                                                                                
        // Update previous variables
        error_dist_left_old = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_angle_old = error_angle;
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
        setLeftPWM(200 + angleOut);
        setRightPWM(200 - angleOut);

        error_angle_old = error_angle;
        t_old = micros();
    }
}
