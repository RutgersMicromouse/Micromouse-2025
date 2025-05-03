#pragma once

#include <string>

// #define SIM
#define REAL

// define physical libraries if real
#ifdef REAL

#include "frontdist.h"
#include "pidrotate.h"
#include "pidstraight.h"
#include "imu.h"
#include "ioexpander.h"
double block_length = 180;

#endif

class API {

public:


    static bool wallFront();
    static bool wallRight();
    static bool wallLeft();

    static void turnRight();
    static void turnLeft();
    static void moveForward(int distance = 1);
    static void moveForwardHalf(double numHalfSteps = 1);


// define simulator functions if sim
#ifdef SIM 
    static void turnRight45();
    static void turnLeft45();

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
