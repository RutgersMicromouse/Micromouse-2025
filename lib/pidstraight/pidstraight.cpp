#include "pidstraight.h"
//PID for distance
double Kp_dist = 20;
double Ki_dist = 0;
double Kd_dist = 0;
//PID for angle offset
double Kp_angle = 0.5;
double Ki_angle = 0;
double Kd_angle = 0;

bool diag_support = true; // Toggle on 8 goal angle v.s. 4 possible angles
double identity_diag[8] = {0.0,45,90,135,180,225,270,315};
double identity_manhatan[4] = {0.0,90,180,270};

void pidForward(double distance) {
    Serial.println("Hello pidForward!");
    Serial.println(encLeft.read()); Serial.println(encRight.read());
distance = 840 * distance /( WHEEL_DIAM * PI); // Converts mm -> encoder ticks
encLeft.write(0); encRight.write(0); // Reset encoder position

double goal_angle;
if (diag_support) {
    Serial.println("Hello diag support!");
    int index = 0; 
    double arr_diag[8];
    for (int i = 0; i <= 7;i++) {
        arr_diag[i] = fmod(identity_diag[i] - angle(),360);
        if(arr_diag[i] < arr_diag[index]){
            index = i;
        };              
    }
    double goal_angle = identity_diag[index];
} else {
    Serial.println("Hello manhattan angles!");
    int index = 0;
    double arr_diag[4];
    for (int i=0; i<=4; i++) {
        arr_diag[i] = fmod(identity_manhatan[i] - angle(),360);
        if(arr_diag[i] < arr_diag[index]){
            index = i;
        };
    }  
    double goal_angle = identity_manhatan[index];
}

double error_int_dist;
double error_deriv_dist;
double error_old_dist = distance;
double error_dist=1; 
double t_i = micros();
double out;
double error_int_angle;
double error_deriv_angle;
double error_old_angle = distance;
double error_angle; 
double sampleTime = micros();
double sampleRight = encRight.read();
double sampleLeft = encLeft.read();
while (true) {
//Guard Clauses
    if (error_old_dist == 0.0) {setRightPWM(0); setLeftPWM(0);return;} //At the destination
    if (micros() > sampleTime + 50e3){
        if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2)
            {setRightPWM(0);setLeftPWM(0);return;};
    }; //Stall Condition
    if(front() < 100) {
        setRightPWM(0);setLeftPWM(0);return;
    } //Close to the wall
//End Guard Clauses
    Serial.println("Hello error loop!");
    error_dist = distance - (encLeft.read() + encRight.read())/2; 
    error_angle = goal_angle - angle();

    error_int_dist += error_dist * (micros() - t_i);
    error_int_angle += error_angle * (micros() - t_i);

    error_deriv_dist = (error_dist- error_old_dist)/(micros()-t_i);
    error_deriv_angle = (error_angle - error_old_angle)/(micros()-t_i);

    out = Kp_dist * error_dist + Ki_dist * error_int_dist + Kd_dist * error_deriv_dist;
    double Angle_out = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
    setLeftPWM(out - Angle_out); setRightPWM(out + Angle_out); 
    Serial.println(encLeft.read());Serial.println(encRight.read());
    error_old_dist = error_dist; error_old_angle = error_angle; t_i = micros();
};}

double pidForwardUntil(char condition) {
Serial.println("Hello PidForwardUntil!");
double a=1.0;
return a;
}