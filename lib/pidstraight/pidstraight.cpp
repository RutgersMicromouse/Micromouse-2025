#include "pidstraight.h"
//PID for distance
double Kp_dist = 0.45;
double Ki_dist = 0;
double Kd_dist = 0;
//PID for angle offset
double Kp_angle = 0;
double Ki_angle = 0;
double Kd_angle = 0;

bool diag_support = true; // Toggle on 8 goal angle v.s. 4 possible angles
double identity_diag[8] = {0.0,45,90,135,180,225,270,315};
double identity_manhattan[4] = {0.0,90,180,270};

void pidForward(double distance) {
    Serial.println("Hello pidForward!");
    Serial.println(encLeft.read()); Serial.println(encRight.read());
    double goal_distance = TICKS_PER_ROTATION * distance /( WHEEL_DIAM * PI); // Converts mm -> encoder ticks
    encLeft.write(0); encRight.write(0); // Reset encoder position

    double goal_angle;
    if (diag_support) {
        Serial.println("Hello diag support!");
        int index = 0; 
        double arr_diag[8];
        for (int i = 0; i <= 7; i++) {
            arr_diag[i] = fmod(identity_diag[i] - angle(), 360);
            if(arr_diag[i] < arr_diag[index]) {
                index = i;
            }  
        }
        double goal_angle = identity_diag[index];
    } else {
        Serial.println("Hello manhattan angles!");
        int index = 0;
        double arr_diag[4];
        for (int i = 0; i <= 4; i++) {
            arr_diag[i] = fmod(identity_manhattan[i] - angle(), 360);
            if(arr_diag[i] < arr_diag[index]) {
                index = i;
            }
        }  
        double goal_angle = identity_manhattan[index];
    }

    double error_dist = goal_distance; 
    double error_int_dist;
    double error_deriv_dist;
    double error_dist_old = goal_distance;
    double t_old = micros();
    
    double out;

    double error_int_angle;
    double error_deriv_angle;
    double error_angle_old = fmod(goal_angle - angle(), 360);
    double error_angle; 
    
    // sampling to check for motor stalling
    double sampleTime = micros();
    double sampleRight = encRight.read();
    double sampleLeft = encLeft.read();
    
    
    while (true) {
        // Guard Clauses:
        // 1. At the destination
        if (abs(error_dist) <= 20) { setRightPWM(0); setLeftPWM(0); return; }
        
        // 2. Stall Condition
        if (micros() > sampleTime + 50e3){
            if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2)
                                   { setRightPWM(0); setLeftPWM(0); return; };
        }

        // 3. Too close to the front wall
        if(front() < 100)          { setRightPWM(0); setLeftPWM(0); return; }
        //End Guard Clauses

        Serial.println("Hello error loop!");
        error_dist = goal_distance - (encLeft.read() + encRight.read())/2; 
        error_angle = fmod(goal_angle - angle(), 360);

        error_int_dist += error_dist * (micros() - t_old);
        error_int_angle += error_angle * (micros() - t_old);

        error_deriv_dist = (error_dist - error_dist_old)/(micros() - t_old);
        error_deriv_angle = (error_angle - error_angle_old)/(micros() - t_old);

        out = Kp_dist * error_dist + Ki_dist * error_int_dist + Kd_dist * error_deriv_dist;
        double angle_out = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
        setLeftPWM(out - angle_out); setRightPWM(out + angle_out); 


        Serial.print(encLeft.read()); Serial.print(" "); Serial.println(encRight.read());

        // update error_dist_old, error_angle_old, and t_old
        error_dist_old = error_dist; error_angle_old = error_angle; t_old = micros();
    };}

    void pidForwardLeftWallFollow() {
    Serial.println("Hello pidForwardLeftWallFollow!");

    return;
}