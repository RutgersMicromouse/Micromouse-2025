#pragma once

#include <string>
#include "../Motors/Motors.h"
#include "../DistanceSensors/distanceSensors.h"
#include "../IMU/IMU.h"
#include "../pidRotation/pidRotate.h"
#include "../pidRotation/pidStraight.h"

class API {

public:

    static int mazeWidth();
    static int mazeHeight();

    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward(int distance = 1, uint8_t direction = 255);
    static void moveForwardHalf(int distance = 1, uint8_t direction = 255);
    static void turnRight(uint8_t direction);
    static void turnLeft(uint8_t direction);
    static void turnRight45(uint8_t direction);
    static void turnLeft45(uint8_t direction);

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