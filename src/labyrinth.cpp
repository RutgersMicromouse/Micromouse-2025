#include "labyrinth.h"

void labyrinth() {

    while(true) {

        if(!API::wallLeft()) {

            API::turnLeft();
            API::moveForwardLab();

        } else if(!API::wallFront()) {

            API::moveForwardLab();

        } else {

            API::turnRight();
        }
    }
}