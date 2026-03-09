#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Latest sensor readings — written by sensorTask, read by everything else
extern volatile double   g_angle;
extern volatile int16_t  g_frontDist;
extern volatile bool     g_leftWall;   // true = left wall detected  (IO expander P7)
extern volatile bool     g_rightWall;  // true = right wall detected (IO expander P6)

// The I2C mutex — must be held before ANY I2C call
extern SemaphoreHandle_t i2cMutex;
