#include "labyrinth.h"

void labyrinthLoop() {
    while(true) {
        pidForwardLeftWallFollow();
        if (!API::wallLeft()) API::turnLeft();
        else if (API::wallFront()) API::turnRight();    
    }
}