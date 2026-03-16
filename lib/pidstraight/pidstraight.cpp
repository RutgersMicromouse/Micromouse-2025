#include "pidstraight.h"

// PID for distance
double Kp_dist = 0.3;
double Ki_dist = 0;
double Kd_dist = 0;

// PID for angle offset
double Kp_angle = 3.63;
double Ki_angle = -0.2;
double Kd_angle = 0.01;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

// Target forward speed in encoder ticks per second.
// TICKS_PER_MM = TICKS_PER_ROTATION / (PI * WHEEL_DIAM) = 840 / (PI * 32) ≈ 8.36 ticks/mm
// GOAL_SPEED_TICKS_S = 500 mm/s * (840 / (PI * 32)) ≈ 4178 ticks/s
#define GOAL_SPEED_MM_S    500.0
#define TICKS_PER_MM       (TICKS_PER_ROTATION / (PI * WHEEL_DIAM))
#define GOAL_SPEED_TICKS_S (GOAL_SPEED_MM_S * TICKS_PER_MM)

struct encodercontext {
    int32_t left_position;
    int32_t right_position;
    uint32_t last_update_time;
};

void get_encoderspeed(encodercontext* old, encodercontext* newcontext, double* left_speed, double* right_speed) {
    double dt_s = (newcontext->last_update_time - old->last_update_time) / 1e6;
    *left_speed  = (newcontext->left_position  - old->left_position)  / dt_s;
    *right_speed = (newcontext->right_position - old->right_position) / dt_s;
    memcpy(old, newcontext, sizeof(encodercontext));
}

// Drive forward `distance` mm using full PID speed control and heading hold
void pidForward(double distance) {
    // Convert target distance (mm) to encoder ticks
    int32_t target_ticks = (int32_t)(distance * TICKS_PER_MM);

    encodercontext old_encoder, new_encoder;
    old_encoder.left_position  = encLeft.read();
    old_encoder.right_position = -encRight.read();
    old_encoder.last_update_time = micros();
    int32_t start_left  = old_encoder.left_position;
    int32_t start_right = old_encoder.right_position;

    double left_speed = 0, right_speed = 0;

    // Speed PID state
    double speed_error_int = 0, speed_error_old = 0;

    // Angle PID state: hold the heading captured at entry
    double goal_angle = angle();
    double angle_error_int = 0, angle_error_old = 0;

    while (true) {
        new_encoder.left_position  = encLeft.read();
        new_encoder.right_position = -encRight.read();
        new_encoder.last_update_time = micros();

        // Exit once the average of both encoders reaches the target distance
        int32_t traveled = ((new_encoder.left_position  - start_left) +
                            (new_encoder.right_position - start_right)) / 2;
        if (traveled >= target_ticks) {
            setLeftPWM(0);
            setRightPWM(0);
            return;
        }

        uint32_t dt_us = new_encoder.last_update_time - old_encoder.last_update_time;
        if (dt_us == 0) continue; // skip if no time has elapsed
        double dt_s = dt_us / 1e6;

        get_encoderspeed(&old_encoder, &new_encoder, &left_speed, &right_speed);

        // Speed PID: regulate average wheel speed to GOAL_SPEED_TICKS_S
        double avg_speed   = (left_speed + right_speed) / 2.0;
        double speed_error = GOAL_SPEED_TICKS_S - avg_speed;
        double speed_error_deriv = (speed_error - speed_error_old) / dt_s;
        speed_error_old    = speed_error;
        double base_pwm    = Kp_dist * speed_error
                           + Ki_dist * speed_error_int
                           + Kd_dist * speed_error_deriv;
        // Anti-windup: only integrate when output is not saturated
        if (base_pwm >= -200.0 && base_pwm <= 200.0) {
            speed_error_int += speed_error * dt_s;
        }
        base_pwm = constrain(base_pwm, -200, 200);

        // Angle PID: hold the initial heading
        double angle_error = goal_angle - angle();
        if (angle_error >  180) angle_error -= 360;
        if (angle_error < -180) angle_error += 360;
        double angle_error_deriv = (angle_error - angle_error_old) / dt_s;
        angle_error_old    = angle_error;
        double angle_out   = Kp_angle * angle_error
                           + Ki_angle * angle_error_int
                           + Kd_angle * angle_error_deriv;
        // Anti-windup: only integrate when output is not saturated
        if (angle_out >= -100.0 && angle_out <= 100.0) {
            angle_error_int += angle_error * dt_s;
        }
        angle_out = constrain(angle_out, -100, 100);

        setLeftPWM((int)constrain(base_pwm  + angle_out, -200, 200));
        setRightPWM((int)constrain(base_pwm - angle_out, -200, 200));
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
