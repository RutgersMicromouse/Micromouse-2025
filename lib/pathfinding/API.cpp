#include "./API.h"

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

int API::mazeWidth() {
    //std::cout << "mazeWidth" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

int API::mazeHeight() {
    //std::cout << "mazeHeight" << std::endl;
    std::string response;
    std::cin >> response;
    return atoi(response.c_str());
}

bool API::wallFront() {

    int16_t distance = checkFrontWall();

    if(distance < 80) {
        Serial.printf("front wall detected: true\n");
        return true;
    } else {
        return false;
    }

}

bool API::wallRight() {

    boolean checkRight = checkRightWall();    

    if(checkRight) {
        Serial.printf("right wall detected: true\n");
        return true;
    } else {
        return false;
    }
 
}

bool API::wallLeft() {
    boolean checkLeft = checkLeftWall();    

    if(checkLeft) {
        Serial.printf("left wall detected: true\n");
        return true;
    } else {
        return false;
    }
}

void API::moveForward(int distance, uint8_t direction) {
    // Serial.println(direction);
    //std::cout << "moveForward ";
    // Don't print distance argument unless explicitly specified, for
    // backwards compatibility with older versions of the simulator
    if (distance != 1) {
        //std::cout << distance;
    }

    char dirGoal;

    distance = distance * blockLength;

    if(direction == N) {
        dirGoal = 'N';
    } else if(direction == E) {
        dirGoal = 'E';
    } else if(direction == S) {
        dirGoal = 'S';
    } else {
        dirGoal = 'W';
    }

    Serial.printf("direction %c", dirGoal);
    while(1);
    straight(dirGoal, 160);

    //std::cout << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << response << std::endl;
        throw;
    }
}

void API::moveForwardHalf(int distance, uint8_t direction) {
    //std::cout << "moveForwardHalf ";
    // Don't print distance argument unless explicitly specified, for
    // backwards compatibility with older versions of the simulator
    if (distance != 1) {
        //std::cout << distance;
    }

    char dirGoal;

    distance = distance * blockLength;

    if(direction == N) {
        dirGoal = 'N';
    } else if(direction == E) {
        dirGoal = 'E';
    } else if(direction == S) {
        dirGoal = 'S';
    } else {
        dirGoal = 'W';
    }

    straight(dirGoal, distance);

    //std::cout << std::endl;
    std::string response;
    std::cin >> response;
    if (response != "ack") {
        std::cerr << response << std::endl;
        throw;
    }
}

void API::turnRight(uint8_t direction) {

    char dirGoal;

    if(direction == N) {
        dirGoal = 'E';
    } else if(direction == E) {
        dirGoal = 'S';
    } else if(direction == S) {
        dirGoal = 'W';
    } else {
        dirGoal = 'N';
    }

    turnTo(dirGoal);

    //std::cout << "turnRight" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnLeft(uint8_t direction) {
    char dirGoal;

    if(direction == N) {
        dirGoal = 'W';
    } else if(direction == E) {
        dirGoal = 'N';
    } else if(direction == S) {
        dirGoal = 'E';
    } else {
        dirGoal = 'S';
    }

    turnTo(dirGoal);

    //std::cout << "turnLeft" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnRight45(uint8_t direction) {
    //std::cout << "turnRight45" << std::endl;
    std::string ack;
    std::cin >> ack;
}

void API::turnLeft45(uint8_t direction) {
    //std::cout << "turnLeft45" << std::endl;
    std::string ack;
    std::cin >> ack;
}
void API::setWall(int x, int y, char direction) {
    //std::cout << "setWall " << x << " " << y << " " << direction << std::endl;
}

void API::clearWall(int x, int y, char direction) {
    //std::cout << "clearWall " << x << " " << y << " " << direction << std::endl;
}

void API::setColor(int x, int y, char color) {
    //std::cout << "setColor " << x << " " << y << " " << color << std::endl;
}

void API::clearColor(int x, int y) {
    //std::cout << "clearColor " << x << " " << y << std::endl;
}

void API::clearAllColor() {
    //std::cout << "clearAllColor" << std::endl;
}

void API::setText(int x, int y, const std::string& text) {
    //std::cout << "setText " << x << " " << y << " " << text << std::endl;
}

void API::clearText(int x, int y) {
    //std::cout << "clearText " << x << " " << y << std::endl;
}

void API::clearAllText() {
    //std::cout << "clearAllText" << std::endl;
}

bool API::wasReset() {
    //std::cout << "wasReset" << std::endl;
    std::string response;
    std::cin >> response;
    return response == "true";
}

void API::ackReset() {
    //std::cout << "ackReset" << std::endl;
    std::string ack;
    std::cin >> ack;
}