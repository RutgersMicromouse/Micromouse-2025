#include "pidstraight.h"
//PID for distance
double Kp_dist = 0.35;
double Ki_dist = 0;
double Kd_dist = 0;
//PID for angle offset
double Kp_angle = 3;
double Ki_angle = 0;
double Kd_angle = 0;

double identity_diag[8] = {0.0,45,90,135,180,225,270,315};

// Distance forward in mm
void pidForward(double distance) {
    Serial.print("Hello pidForward! ");
    Serial.print(encLeft.read()); Serial.print(" "); Serial.println(encRight.read());
    double goal_distance = TICKS_PER_ROTATION * distance /( WHEEL_DIAM * PI); // Converts mm -> encoder ticks
    goal_distance *= 1.10;
    encLeft.write(0); encRight.write(0); // Reset encoder position

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

    double error_dist_left = goal_distance; double error_int_dist_left; double error_deriv_dist_left;
    double error_dist_left_old = error_dist_left;

    double error_dist_right = goal_distance; double error_int_dist_right; double error_deriv_dist_right;
    double error_dist_right_old = error_dist_right; 
    
    double error_angle = goal_angle - angle();
    // angle wrapping
    if (error_angle > 180) error_angle -= 360;
    if (error_angle < -180) error_angle += 360;
    
    double error_int_angle; double error_deriv_angle;
    double error_angle_old = error_angle;
    
    double distOutLeft;
    double distOutRight;
    double angleOut;

    // sampling to check for motor stalling
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();
    
    
    while (true) {
    
        // Guard Clauses:
        // 1. At the destination
        if (abs(error_dist_left) <= 3 && abs(error_dist_right) <= 3) { setRightPWM(0); setLeftPWM(0); return; }
        
        // 2. Stall Condition, 0.1 second
        if (micros() > sampleTime + 1e5){
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { setRightPWM(0); setLeftPWM(0); return; }
            
            // Update stall sampler
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
        }

        // 3. Too close to the front wall
        if(front() < 25)          { setRightPWM(0); setLeftPWM(0); return; }

        // P error
        error_dist_left = goal_distance - encLeft.read(); 
        error_dist_right = goal_distance - encRight.read(); 
        error_angle = goal_angle - angle();
        // angle wrapping
        if (error_angle > 180) error_angle -= 360;
        if (error_angle < -180) error_angle += 360;
        
        // I error
        error_int_dist_left += error_dist_left * (micros() - t_old);
        error_int_dist_right += error_dist_right * (micros() - t_old);
        error_int_angle += error_angle * (micros() - t_old);
        
        // D error
        error_deriv_dist_left = (error_dist_left - error_dist_left_old)/(micros() - t_old);
        error_deriv_dist_right = (error_dist_right - error_dist_right_old)/(micros() - t_old);
        error_deriv_angle = (error_angle - error_angle_old)/(micros() - t_old);

        distOutLeft = Kp_dist * error_dist_left + Ki_dist * error_int_dist_left + Kd_dist * error_deriv_dist_left;
        distOutRight = Kp_dist * error_dist_right + Ki_dist * error_int_dist_right + Kd_dist * error_deriv_dist_right;
        angleOut = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
        Serial.printf("angleOut %f \n", angleOut);
        setRightPWM(distOutRight - angleOut); delay(10); setLeftPWM(distOutLeft + angleOut);

        Serial.print(encLeft.read()); Serial.print(" "); Serial.println(encRight.read());

        // update error_dist_old, error_angle_old, and t_old
        error_dist_left_old = error_dist_left; error_dist_right_old = error_dist_right; error_angle_old = error_angle; t_old = micros();
    }
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
        if (!leftWall()) { setRightPWM(0); setLeftPWM(0); return; }
        
        // 2. Stall Condition, 0.1 second
        if (micros() > sampleTime + 1e5){
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2) { setRightPWM(0); setLeftPWM(0); return; }
            
            // Update stall sampler
            sampleTime = micros();
            sampleRight = encRight.read();
            sampleLeft = encLeft.read();
        }

        // 3. Too close to the front wall
        if(front() < 100)          { setRightPWM(0); setLeftPWM(0); return; }

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
        setLeftPWM(400 + angleOut); setRightPWM(400 - angleOut); 


        // update error_angle_old, and t_old
        error_angle_old = error_angle; t_old = micros();
    }
}