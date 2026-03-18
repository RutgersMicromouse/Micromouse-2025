#pragma once
#include <Arduino.h>
#include "imu.h"
#include "motors.h"

void straight(char direction, int distance);
void straightASTAR(char direction);
int16_t leftWallDistance();
int16_t rightWallDistance();
double getDistError();