#include "labyrinth.h"

void labyrinthLoop() {
    while(true) {

        if (!API::wallLeft()) {
            API::turnLeft();
            API::moveForward();
            pidForwardLeftWallFollow();  // move after turn
        } else if (!API::wallFront()) {
            pidForwardLeftWallFollow();  // no wall ahead, keep going
        } else {
            API::turnRight();  // wall left and front → dead end
        }

    }

}