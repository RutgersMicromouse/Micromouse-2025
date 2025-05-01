#pragma once

#include <string>
#include "../lib/Motors/Motors.h"
#include "../lib/DistanceSensors/distanceSensors.h"
#include "../lib/IMU/IMU.h"
#include "../lib/pidRotation/pidRotate.h"
#include "../lib/pidRotation/pidStraight.h"

class API {

public:

    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward(int distance = 160);
    static void moveForwardHalf(int numHalfSteps = 80);
    static void turnRight();
    static void turnLeft();
    static void turnRight45();
    static void turnLeft45();

    static void setWall(int x, int y, char direction);
    static void clearWall(int x, int y, char direction);

    static void setColor(int x, int y, char color);
    static void clearColor(int x, int y);
    static void clearAllColor();

    static void setText(int x, int y, const std::string& text);
    static void clearText(int x, int y);
    static void clearAllText();

    static bool wasReset();
    static void ackReset();

};