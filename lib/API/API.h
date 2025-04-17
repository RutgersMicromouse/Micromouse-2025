#pragma once

#include <string>

#define SIM
// #define REAL

// define physical libraries if real
#ifdef REAL

#include "..\src\lib\distance_sensor.h"
#include "..\src\lib\pidRotate.h"
#include "..\src\lib\pidStraight.h"
#include "..\src\lib\IMU.h"

#endif

class API {

public:


    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void moveForward(int distance = 1);
    static void moveForwardHalf(int numHalfSteps = 1);
    static void turnRight();
    static void turnLeft();
    static void turnRight45();
    static void turnLeft45();

// define simulator functions if sim
#ifdef SIM 
    static int mazeWidth();
    static int mazeHeight();
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
#endif
};
