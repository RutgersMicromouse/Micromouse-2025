#pragma once

#define PI 3.1415926535897932384626433832795
#define WHEEL_DIAM 32 // wheel diameter in mm



#include "imu.h"
#include "motors.h"
#include "frontdist.h"

// use PID to drive forward in mm
void pidForward(double distance);

// use PID to drive straight until some condition(s)
// returns distance driven in mm
double pidForwardUntil(char condition);