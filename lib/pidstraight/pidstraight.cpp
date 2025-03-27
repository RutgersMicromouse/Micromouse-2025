#include "pidstraight.h"
//PID for distance
double Kp_dist = 0;
double Ki_dist = 0;
double Kd_dist = 0;
//PID for angle offset
double Kp_angle = 0;
double Ki_angle = 0;
double Kd_angle = 0;
bool diag_support = True;
double identity_diag[8] = {0,45,90,135,180,225,270,315};

void pidForward(double distance) {
distance = 840 * distance /( WHEEL_DIAM * PI); // Converts mm -> encoder ticks
encLeft.write(0); encRight.write(0) // Reset encoder position

double goal_angle;
if diag_support{
for int i = 0:7 {
arr_diag[i] = (identity_diag[i] - current_angle) % 360;
}

double goal_angle = identity_arr[min(arr_diag)];
}

double error_int_dist;
double error_deriv_dist;
double error_old_dist = distance;
double error_dist; 
double t_i = micros();
double out;
double error_int_angle;
double error_deriv_angle;
double error_old_angle = distance;
double error_angle; 

while True {
//Guard Clauses
    if (error = 0){setRightPWM(0);setLeftPWM(0);return}; //At the destination
    if (micros() > sampleTime + 50e3){
        if (abs(encRight.read() - sampleRight) < 2 || abs(encLeft.read() - sampleLeft) < 2)
            {setRightPWM(0);setLeftPWM(0);return};
    }; //Stall Condition
    if(front() < 15) {
        setRightPWM(0);setLeftPWM(0);return;
    } //Close to the wall
//End Guard Clauses

    error_dist = distance - (encLeft.read() + encRight.read())/2; 
    error_angle = 

    error_int_dist += error_dist * (micros() - t_i);
    error_int_angle += error_angle * (micros() - t_i);

    error_deriv_dist = (error_dist- error_old_dist)/(micros()-t_i);
    error_deriv_angle = (error_angle - error_old_angle)/(micros()-t_i);

    out = Kp_dist * error_dist + Ki_dist * error_int_dist + Kd_dist * error_deriv_dist;
    Angle_out = Kp_angle * error_angle + Ki_angle * error_int_angle + Kd_angle * error_deriv_angle;
    setLeftPWM(out - Angle_out); setRightPWM(out + Angle_out); 
    error_old_dist = error_dist; error_old_angle = error_angle; t_i = micros();
}




double pidForwarrror == 0dUntil(char condition) {


}