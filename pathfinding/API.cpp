#include "./API.h"

#include <cstdlib>
#include <iostream>

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

bool API::wallFront() {

    int16_t distance = checkFrontWall();

    if(distance < 30) {
        std::cout << "wallFront" << std::endl;
        return true;
    } else {
        return false;
    }

}

bool API::wallRight() {

    boolean checkRight = checkRightWall();    

    if(checkRight) {
        std::cout << "wallRight" << std::endl;
        return true;
    } else {
        return false;
    }
 
}

bool API::wallLeft() {
    boolean checkLeft = checkLeftWall();    

    if(checkLeft) {
        std::cout << "wallLeft" << std::endl;
        return true;
    } else {
        return false;
    }
}

void API::moveForward(int distance) {
    std::cout << "moveForward ";
    // Don't print distance argument unless explicitly specified, for
    // backwards compatibility with older versions of the simulator
    if (distance != 1) {
        std::cout << distance;
    }

    straight('N', distance);

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

    straight('N', numHalfSteps);

    std::cout << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << response << std::endl;
        throw;
    }
}

void API::turnRight() {
    char angle_goal;

    double currentAngle = angle();
    if(currentAngle < 45 || currentAngle > 315) {
        angle_goal = 'E';
    } else if(currentAngle > 45 && currentAngle < 135) {
        angle_goal = 'S';
    } else if(currentAngle > 135 && currentAngle < 225) {
        angle_goal = 'W';
    } else {
        angle_goal = 'N';
    }

    turnTo(angle_goal);

    std::cout << "turnRight" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnLeft() {
    char angle_goal;

    double currentAngle = angle();
    if(currentAngle < 45 || currentAngle > 315) {
        angle_goal = 'W';
    } else if(currentAngle > 45 && currentAngle < 135) {
        angle_goal = 'N';
    } else if(currentAngle > 135 && currentAngle < 225) {
        angle_goal = 'E';
    } else {
        angle_goal = 'S';
    }

    turnTo(angle_goal);


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