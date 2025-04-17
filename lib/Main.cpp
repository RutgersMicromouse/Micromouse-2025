#include "API.h"

#ifdef SIM

#include <iostream>
#include <string>
#include <stack>
#include <algorithm>

#include "Flood.h"
using namespace std;


void log(const std::string& text) {
    std::cerr << text << std::endl;
}

int main(int argc, char* argv[]) {
    log("Running...");
    API::setColor(0, 0, 'G');
    API::setText(0, 0, "abc");

    // check size of 16x16 maze and walls array for EEPROM
    //log(to_string(sizeof(maze)));
    //log(to_string(sizeof(walls)));
   
    //// STEP 1: Initial Solve
    initialize();
   	 
    //Modified Flood Fill
    for (int run = 1; run <= 5; run++) {
	    std::cerr << "Run #" << run << std::endl;
	    runMaze('c');
	    backTrack();
    }

    // A*
    std::cerr << "Speedrun" << std::endl;
    speedrun();

}

#endif
