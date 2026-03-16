#pragma once

#define PI 3.1415926535897932384626433832795
#define WHEEL_DIAM 12 // wheel diameter in mm
#define TICKS_PER_ROTATION 100 // 840 encoder ticks per 1 wheel rotation



#include "imu.h"
#include "motors.h"
#include "frontdist.h"
#include "ioexpander.h"

// use PID to drive forward in mm
void pidForward(double distance);

// left wall following logic
/*  
    labyrinth.cpp
    while(true) {
        pidForwardLeftWallFollow();
        if (!wallLeft()) turnLeft();
        else if (wallFront()) turnRight();    
    }
*/

// use PID to drive straight infinitely until condition
// if (!wallLeft() || wallFront()) { return };

void pidForwardLeftWallFollow();