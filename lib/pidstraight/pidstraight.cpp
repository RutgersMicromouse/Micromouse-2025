#include "pidstraight.h"
#include "shared.h"
#include <cmath>

// PID for distance
double Kp_dist = 0.3;
double Ki_dist = 0;
double Kd_dist = 0;

// PID for IMU angle (used when no walls, or both walls present)
double Kp_angle = 3.63;
double Ki_angle = -0.2;
double Kd_angle = 0.01;

// Fixed heading offset (degrees) applied when only ONE wall is detected.
// Small enough to be smooth, large enough to pull the robot back toward center.
// Increase if robot still drifts into wall; decrease if it oscillates side to side.
#define WALL_NUDGE_DEG 3.0

// Distance forward in mm
void pidForward(double distance) {
    Serial.print("Hello pidForward! ");
    double goal_distance = TICKS_PER_ROTATION * distance / (WHEEL_DIAM * PI);
    Serial.print("Goal distance: ");
    Serial.println(goal_distance);

    // Reset encoders
    encLeft.write(0);
    encRight.write(0);

    // Build heading targets relative to calibrated north (set at startup with both walls present).
    // This means 0° in our grid = the exact angle the robot was at when placed in the start corridor.
    double identity_diag[8];
    for (int i = 0; i < 8; i++) {
        identity_diag[i] = fmod(g_calibratedNorth + i * 45.0, 360.0);
        if (identity_diag[i] < 0) identity_diag[i] += 360.0;
    }

    // Snap to the nearest calibrated 45° heading
    int closest_index = 0;
    double arr_diag[8];
    for (int i = 0; i <= 7; i++) {
        arr_diag[i] = identity_diag[i] - g_angle;
        if (arr_diag[i] >  180) arr_diag[i] -= 360;
        if (arr_diag[i] < -180) arr_diag[i] += 360;
        if (abs(arr_diag[i]) < abs(arr_diag[closest_index])) closest_index = i;
    }
    double goal_angle = identity_diag[closest_index];
    Serial.print("Goal angle: ");
    Serial.println(goal_angle);

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

    double error_angle = goal_angle - g_angle;
    if (error_angle >  180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;
    double error_int_angle  = 0;
    double error_deriv_angle = 0;
    double error_angle_old  = error_angle;

    double distOutLeft = 0;
    double distOutRight = 0;

    // Stall detection setup
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();

    // Acceleration ramp
    double startFactor = 0.1;
    double accelRate = 3;
    double rampProgress = 0.0;

    while (true) {
        // --- Guard Clauses ---
        // Don't stop motors — PID output is ~0 when error is this small.
        // Returning without stopping lets the next command take over immediately.
        if (abs(error_dist_left) <= 3 && abs(error_dist_right) <= 3) {
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

        // Stop if too close to front wall
        if (g_frontDist < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }

        // --- PID error calculations ---
        error_dist_left  = goal_distance - encLeft.read();
        error_dist_right = goal_distance - encRight.read();
        error_angle = goal_angle - g_angle;
        if (error_angle >  180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        double t_now = micros();
        double dt = (t_now - t_old) / 1e6;
        if (dt <= 0) dt = 1e-3;

        error_int_dist_left  += error_dist_left * dt;
        error_int_dist_right += error_dist_right * dt;
        error_int_angle      += error_angle * dt;

        error_deriv_dist_left  = (error_dist_left  - error_dist_left_old)  / dt;
        error_deriv_dist_right = (error_dist_right - error_dist_right_old) / dt;
        error_deriv_angle      = (error_angle - error_angle_old) / dt;

        distOutLeft  = Kp_dist * error_dist_left  + Ki_dist * error_int_dist_left  + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        double angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;

        // --- Wall bias ---
        // Apply a fixed heading offset based on which walls are visible.
        // This is re-applied fresh every loop — no accumulation, no clamp needed.
        // Both walls or no walls → drive straight (base heading).
        // One wall only → steer 3° away from it.
        double wall_bias = 0.0;
        if      (g_leftWall && !g_rightWall)  wall_bias = -WALL_NUDGE_DEG; // left wall only → steer right
        else if (!g_leftWall && g_rightWall)  wall_bias =  WALL_NUDGE_DEG; // right wall only → steer left

        error_angle = (identity_diag[closest_index] + wall_bias) - g_angle;
        if (error_angle >  180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;

        double rightPWM = (distOutRight - angleOut) * 1.25;
        double leftPWM  = (distOutLeft  + angleOut) * 1.25;

        // Acceleration ramp
        double avgEncoder = (abs(encLeft.read()) + abs(encRight.read())) / 2.0;
        rampProgress = constrain(avgEncoder / goal_distance, 0.0, 1.0);
        double rampFactor = startFactor + (1.0 - startFactor) * (1.0 - exp(-accelRate * rampProgress));

        rightPWM *= rampFactor;
        leftPWM  *= rampFactor;

        rightPWM = constrain(rightPWM, -400, 400);
        leftPWM  = constrain(leftPWM,  -400, 400);

        setRightPWM(rightPWM);
        setLeftPWM(leftPWM);

        Serial.print("Ramp: ");        Serial.print(rampFactor, 2);
        Serial.print(" | PWM L/R: ");  Serial.print(leftPWM, 1);
        Serial.print(" / ");           Serial.print(rightPWM, 1);
        Serial.print(" | Wall L/R: "); Serial.print(g_leftWall);
        Serial.print(" / ");           Serial.println(g_rightWall);

        error_dist_left_old  = error_dist_left;
        error_dist_right_old = error_dist_right;
        error_angle_old      = error_angle;
        t_old = t_now;
    }
}


void pidForwardLeftWallFollow() {
    Serial.println("Hello pidForwardLeftWallFollow!");

    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft  = encLeft.read();

    while (true) {
        // Stop when left wall disappears
        if (!g_leftWall) {
            vTaskDelay(pdMS_TO_TICKS(120)); // RTOS-safe delay — doesn't freeze other tasks
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }

        // Stall detection
        if (micros() > sampleTime + 1e5) {
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) {
                setRightPWM(0);
                setLeftPWM(0);
                return;
            }
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft  = encLeft.read();
        }

        // Stop if front wall approaching
        if (g_frontDist < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }

        // Left wall present, right wall absent: steer left (toward left wall)
        // Left wall present, right wall present: go straight
        double wallCorrection = (!g_rightWall) ? -30.0 : 0;

        setLeftPWM (200 + wallCorrection);
        setRightPWM(200 - wallCorrection);
    }
}
