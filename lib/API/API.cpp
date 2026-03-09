#include "API.h"
#include "shared.h"

#include <cstdlib>
#include <iostream>

#ifdef REAL
double block_length = 180;

bool API::wallFront() {
    return g_frontDist < 90;
}

bool API::wallRight() {
    return rightWall();
}

bool API::wallLeft() {
    return leftWall();
}

void API::moveForward(int distance) {
    pidForward(distance * block_length);
}

void API::moveForwardHalf(double numHalfSteps) {
    pidForward(numHalfSteps * block_length/2);
}

// void API::turnRight() {
//     double currentAngle = angle();
//     if (currentAngle > 45 && currentAngle > 135) {currentAngle = 90;}
//     else if (currentAngle > 135 && currentAngle < 225) {currentAngle = 180;}
//     else if (currentAngle > 225 && currentAngle < 315) {currentAngle = 270;}
//     else {currentAngle = 0;}
//     turnTo(currentAngle+90);
// }

// void API::turnLeft() {
//     double currentAngle = angle();
//     if (currentAngle > 45 && currentAngle > 135) {currentAngle = 90;}
//     else if (currentAngle > 135 && currentAngle < 225) {currentAngle = 180;}
//     else if (currentAngle > 225 && currentAngle < 315) {currentAngle = 270;}
//     else {currentAngle = 0;}
//     turnTo(currentAngle-90);
// }

void API::turnRight() {
    double angle_goal;

    double currentAngle = g_angle;
    if(currentAngle < 45 || currentAngle > 315) {
        angle_goal = 90;
    } else if(currentAngle > 45 && currentAngle < 135) {
        angle_goal = 180;
    } else if(currentAngle > 135 && currentAngle < 225) {
        angle_goal = 270;
    } else {
        angle_goal = 0;
    }

    turnTo(angle_goal); //right 90 based on current orientation
    // delay(200);
    return;
}

void API::turnLeft() {
    double angle_goal;

    double currentAngle = g_angle;
    if(currentAngle < 45 || currentAngle > 315) {
        angle_goal = 270;
    } else if(currentAngle > 45 && currentAngle < 135) {
        angle_goal = 0;
    } else if(currentAngle > 135 && currentAngle < 225) {
        angle_goal = 90;
    } else {
        angle_goal = 180;
    }

    angle_goal += 1; // to fix undershoot

    turnTo(angle_goal); //left 90 based on current orientation
    // delay(200);
    return;
}


#endif

#ifdef SIM
bool API::wallFront() {
    std::cout << "wallFront" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

bool API::wallRight() {
    std::cout << "wallRight" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

bool API::wallLeft() {
    std::cout << "wallLeft" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

void API::moveForward(int distance) {
    std::cout << "moveForward ";
    // Don't print distance argument unless explicitly specified, for
    // backwards compatibility with older versions of the simulator
    if (distance != 1) {
        std::cout << distance;
    }
    std::cout << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << response << std::endl;
        throw;
    }
}

void API::moveForwardHalf(int numHalfSteps) {
    std::cout << "moveForwardHalf ";
    // Don't print distance argument unless explicitly specified, for
    // backwards compatibility with older versions of the simulator
    if (numHalfSteps != 1) {
        std::cout << numHalfSteps;
    }
    std::cout << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << response << std::endl;
        throw;
    }
}

void API::turnRight() {
    std::cout << "turnRight" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnLeft() {
    std::cout << "turnLeft" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnRight45() {
    std::cout << "turnRight45" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnLeft45() {
    std::cout << "turnLeft45" << std::endl;
    std::string ack;
    std::cin >> ack;
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
