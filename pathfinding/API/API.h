
#ifndef API_H
#define API_H
#pragma once


#include "../lib/distanceSensor/distance_sensor.h"
#include "../lib/pidRotate/pidRotate.h"
#include "../lib/pidStraight/pidStraight.h"
#include "../lib/IMU/imu.h"

#include <string>

class API {


public:
    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward();
    static void turnRight();
    static void turnLeft();
    static void turnAround();
};
#endif
