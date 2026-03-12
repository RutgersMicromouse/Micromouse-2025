#include "API.h"

#include <cstdlib>
#include <iostream>

#define N 0
#define NE 1
#define E 2
#define SE 3
#define S 4
#define SW 5
#define W 6
#define NW 7

#define blockLength 160

static char dirgoal = 'N';

bool wallFront() {
    int16_t distance = checkFrontWall();

    if(distance < 80) {
        Serial.printf("front wall detected: true\n");
        return true;
    } else {
        return false;
    }
}

bool wallRight() {
    boolean checkRight = checkRightWall();    

    if(checkRight) {
        Serial.printf("right wall detected: true\n");
        return true;
    } else {
        return false;
    }
}

bool wallLeft() {
    boolean checkLeft = checkLeftWall();    

    if(checkLeft) {
        Serial.printf("left wall detected: true\n");
        return true;
    } else {
        return false;
    }
}

void moveForward(int distance) {
    int actualDistance = distance * blockLength;
    straight(dirgoal, actualDistance);
}

void moveForwardHalf(int distance) {
    if (distance != 1) {
        //std::cout << distance;
    }
    int actualDistance = distance * blockLength;
    straight(dirgoal, actualDistance);
}

void turnRight() {
    switch(dirgoal){
        case 'N':
            dirgoal = 'E';
            break;
        case 'E':
            dirgoal = 'S';
            break;
        case 'S':
            dirgoal = 'W';
            break;
        case 'W':
            dirgoal = 'N';
            break;
        default:
            Serial.printf("crash: turn right invalid\n");
            while(1);
    };
    turnTo(dirgoal);
}

void turnLeft() {
    switch(dirgoal){
        case 'N':
            dirgoal = 'W';
            break;
        case 'E':
            dirgoal = 'N';
            break;
        case 'S':
            dirgoal = 'E';
            break;
        case 'W':
            dirgoal = 'S';
            break;
        default:
            Serial.printf("crash: turn left invalid\n");
            while(1);
    };
    turnTo(dirgoal);
}

void turnRight45() {
    // Implementation for turning right by 45 degrees
    // This is a placeholder - actual implementation depends on your robot's control system
}

void turnLeft45() {
    // Implementation for turning left by 45 degrees
    // This is a placeholder - actual implementation depends on your robot's control system
}