#pragma once

#include "imu.h"
#include "motors.h"

// use PID to turn to specific orientations (cardinal NSEW + intermediate directions)
// where North = 0/360, S = 180, E = 90, W = 270
void turnTo(double angleGoal);
void turnToHalf(double goal_angle);