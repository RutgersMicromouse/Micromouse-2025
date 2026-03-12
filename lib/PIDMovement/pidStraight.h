#pragma once
#include <Arduino.h>
#include "../IMU/imu.h"
#include "../Motors/motors.h"

void straight(char direction, int distance);
int16_t leftWallDistance();
int16_t rightWallDistance();
double getDistError();