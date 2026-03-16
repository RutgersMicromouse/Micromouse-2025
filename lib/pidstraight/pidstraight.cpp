#include "pidstraight.h"

// PID for distance
double Kp_dist = 2.5;
double Ki_dist = 0;
double Kd_dist = 0;

// PID for angle offset
double Kp_angle = 0.0;
double Ki_angle = -0.2;
double Kd_angle = 0.01;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

#define goalspeed 150000
struct encodercontext {
    int32_t left_position;
    int32_t right_position;
    uint32_t last_update_time;
    float angle_position;
};

void get_encoderspeed(encodercontext* old, encodercontext* newcontext, double* left_speed, double* right_speed) {
    int32_t dleft = (int32_t)(newcontext->left_position - old->left_position);
    int32_t dright = (int32_t)(newcontext->right_position - old->right_position);
    uint32_t dt_us = newcontext->last_update_time - old->last_update_time;
    double dt = (double)dt_us / 1e6; // seconds
    if (dt < 1e-6) dt = 1e-6;
    *left_speed = 0.25 * ((double)dleft / dt);   // ticks per second
    *right_speed = 0.25 * ((double)dright / dt); // ticks per second
    memcpy(old, newcontext, sizeof(encodercontext));
}

void get_angular_velocity(encodercontext* old, encodercontext* newcontext, double* angular_velocity) {
    double dangle = newcontext->angle_position - old->angle_position;
    // Correct for wrap-around across the -180..180 degree boundary
    if (dangle > 180.0) dangle -= 360.0;
    if (dangle < -180.0) dangle += 360.0;
    uint32_t dt_us = newcontext->last_update_time - old->last_update_time;
    double dt = (double)dt_us / 1e6; // seconds
    if (dt < 1e-6) dt = 1e-6;
    *angular_velocity = dangle / dt; // degrees per second
}

// Distance forward in mm
void pidForward(double distance) {
    encodercontext old_encoder;
    encodercontext new_encoder;
    old_encoder.left_position = encLeft.read();
    old_encoder.right_position = -encRight.read();
    old_encoder.last_update_time = micros();
    old_encoder.angle_position = angle();
    double left_speed, right_speed;
    while(1){
        if(front() < 90) {
            setRightPWM(0);
            setLeftPWM(0);
            return;
        }
        //getting encoder speeds
        new_encoder.left_position = encLeft.read();
        new_encoder.right_position = -encRight.read();
        new_encoder.last_update_time = micros();
        new_encoder.angle_position = angle();
        get_encoderspeed(&old_encoder, &new_encoder, &left_speed, &right_speed);
        // Compute angle error relative to goal 0 degrees and correct for wrap
        double current_angle = (double)new_encoder.angle_position;
        double error_angle = 0.0 - current_angle;
        if (error_angle > 180.0) error_angle -= 360.0;
        if (error_angle < -180.0) error_angle += 360.0;

        // Angle correction (proportional). Tune Kp_angle as needed.
        double angleCorr = Kp_angle * error_angle;

        double error_left = (double)goalspeed - left_speed;
        double error_right = (double)goalspeed - right_speed;

        // Apply angular correction opposite directions on each wheel
        double pwm_left = (Kp_dist * error_left) + angleCorr;
        double pwm_right = (Kp_dist * error_right) - angleCorr;

        // Allow full PWM range; tune limits as needed
        pwm_left = constrain(pwm_left, -150.0, 150.0);
        pwm_right = constrain(pwm_right, -150.0, 150.0);

        setLeftPWM((int)-pwm_left);
        setRightPWM((int)-pwm_right);

        Serial.printf("curr_angle=%lf,AngCorr=%lf\tLspd=%lf\tRspd=%lf\tLpwm=%d\tRpwm=%d\n\r", error_angle, angleCorr, left_speed, right_speed, (int)pwm_left, (int)pwm_right);

        delay(5);
    }
}


void pidForwardLeftWallFollow() {
    Serial.println("Hello pidForwardLeftWallFollow!");

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

    Serial.print("Goal angle: ");
    Serial.println(goal_angle);

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
