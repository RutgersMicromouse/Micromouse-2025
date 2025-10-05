#pragma once

#include <string>
#include "../Motors/motors.h"
#include "../distanceSensors/distanceSensors.h"
#include "../IMU/imu.h"
#include "../PIDMovement/pidRotate.h"
#include "../PIDMovement/pidStraight.h"

bool wallFront();
bool wallRight();
bool wallLeft();
void moveForward(int distance = 1);
void moveForwardHalf(int distance = 0.5);
void turnRight();
void turnLeft();
void turnRight45();
void turnLeft45();