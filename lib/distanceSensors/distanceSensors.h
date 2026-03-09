#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// Front VL53L1X ToF sensor only
// Left/right walls are read via the IO expander (leftWall() / rightWall() in ioexpander.h)
void setupDistanceSensors();
int16_t checkFrontDist();
