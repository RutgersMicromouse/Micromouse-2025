#include "API.h"

#include <cstdlib>
#include <iostream>


// TODO: Add an ifdef REAL to define real functions

#ifdef SIM

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

//Directions for 45 degrees:
    //NE --> A,
    //SE --> B,
    //SW --> C
    //NW --> D

#define blockLength 160

static char dirgoal = 'N';

bool API::wallFront() {

    std::cout << "wallFront" << std::endl;
    int16_t distance = checkFrontWall();
    return distance < 80;
}

bool API::wallRight() {
    std::cout << "wallRight" << std::endl;
    return checkRightWall();
}

bool API::wallLeft() {
    std::cout << "wallLeft" << std::endl;
    return checkLeftWall();
}

void API::moveForward(int distance) {
    std::cout << "moveForward ";
    int actualDistance = distance * blockLength;
    straight(dirgoal, actualDistance);
}

void API::moveForwardHalf(int numHalfSteps) {
    std::cout << "moveForward";
    int actualDistance = 100;
    straight(dirgoal, actualDistance);
}

void API::turnRight() {
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

void API::turnLeft() {
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

void API::turnHalf() {
    switch(dirgoal){
        case 'N':
            dirgoal = 'S';
            break;
        case 'E':
            dirgoal = 'W';
            break;
        case 'S':
            dirgoal = 'N';
            break;
        case 'W':
            dirgoal = 'E';
            break;
        default:
            Serial.printf("crash: turn left invalid\n");
            while(1);
    };
    turnTo(dirgoal);
}


void API::turnRight45() {

    //NE --> A,
    //SE --> B,
    //SW --> C
    //NW --> D

    switch(dirgoal){
        case 'N':
            dirgoal = 'A'; //
            break;
        case 'E':
            dirgoal = 'B';
            break;
        case 'S':
            dirgoal = 'C';
            break;
        case 'W':
            dirgoal = 'D';
            break;
        default:
            Serial.printf("crash: turn left invalid\n");
            while(1);
    };
    turnTo(dirgoal);
}

void API::turnLeft45() {

    //NE --> A,
    //SE --> B,
    //SW --> C
    //NW --> D

    switch(dirgoal){
        case 'N':
            dirgoal = 'D';
            break;
        case 'E':
            dirgoal = 'C';
            break;
        case 'S':
            dirgoal = 'B';
            break;
        case 'W':
            dirgoal = 'A';
            break;
        default:
            Serial.printf("crash: turn left invalid\n");
            while(1);
    };
    turnTo(dirgoal);
}


int API::mazeWidth() {
    std::cout << "mazeWidth" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

int API::mazeHeight() {
    std::cout << "mazeHeight" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

void API::setWall(int x, int y, char direction) {
    std::cout << "setWall " << x << " " << y << " " << direction << std::endl;
}

void API::clearWall(int x, int y, char direction) {
    std::cout << "clearWall " << x << " " << y << " " << direction << std::endl;
}

void API::setColor(int x, int y, char color) {
    std::cout << "setColor " << x << " " << y << " " << color << std::endl;
}

void API::clearColor(int x, int y) {
    std::cout << "clearColor " << x << " " << y << std::endl;
}

void API::clearAllColor() {
    std::cout << "clearAllColor" << std::endl;
}

void API::setText(int x, int y, const std::string& text) {
    std::cout << "setText " << x << " " << y << " " << text << std::endl;
}

void API::clearText(int x, int y) {
    std::cout << "clearText " << x << " " << y << std::endl;
}

void API::clearAllText() {
    std::cout << "clearAllText" << std::endl;
}

bool API::wasReset() {
    std::cout << "wasReset" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

void API::ackReset() {
    std::cout << "ackReset" << std::endl;
    std::string ack;
    std::cin >> ack;
}
#endif
