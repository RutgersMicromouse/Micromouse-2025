#pragma once

#include <string>
#include "../Motors/Motors.h"
#include "../DistanceSensors/distanceSensors.h"
#include "../IMU/IMU.h"
#include "../pidRotation/pidRotate.h"
#include "../pidRotation/pidStraight.h"

bool wallFront();
bool wallRight();
bool wallLeft();
void moveForward(int distance = 1);
void moveForwardHalf(int distance = 1);
void turnRight();
void turnLeft();
void turnRight45();
void turnLeft45();