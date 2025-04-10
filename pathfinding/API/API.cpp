#include "API.h"
#include <cstdlib>
#include <iostream>


bool API::wallFront() {
    int distance = front();
    if(distance < 100) {
        return true;
    } else {
        return false;
    }
}

bool API::wallRight() {
    int distance = right();
    if(distance < 100) {
        return true;
    } else {
        return false;
    }
}

bool API::wallLeft() {
    int distance = left();
    if(distance < 100) {
        return true;
    } else {
        return false;
    }
}


//  pidstraight_instance->InputToMotor(0,blocklength); //go 11 cm. works fine
//   pidrotate_instance->InputToMotor(-90); //left 90
double blockLength = 160;

void API::moveForward() {
    setForwardPWM(blockLength);
    // delay(200);

    // extra distance in case the bot hasn't moved close enough to the front wall (between 30 and 50 mm)
    // try to move 50 mm, will be stopped distance is less than 15 (functionality built into forward pid)
    if(front() >= 30 && front() <= 80) {
            setForwardPWM(20);
    }

    return;
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

    turnTo(angle_goal); //right 90 based on current orientation
    // delay(200);
    return;
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

    turnTo(angle_goal); //left 90 based on current orientation
    // delay(200);
    return;
}

void API::turnAround() {
    char angle_goal;

    double currentAngle = angle();
    if(currentAngle < 45 || currentAngle > 315) {
        angle_goal = 'S';
    } else if(currentAngle > 45 && currentAngle < 135) {
        angle_goal = 'W';
    } else if(currentAngle > 135 && currentAngle < 225) {
        angle_goal = 'N';
    } else {
        angle_goal = 'E';
    }

    turnTo(angle_goal); //left 90 based on current orientation
    // delay(200);
    return;
}
