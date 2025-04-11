#ifndef distance_sensor
#define distance_sensor 
#define rightSensor 4
#define leftSensor 5
#define XSHUT 3

#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>

void setupDistanceSensors();
boolean checkLeftWall();
boolean checkRightWall();
int16_t checkFrontWall();


#endif