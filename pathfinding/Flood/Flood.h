#ifndef FLOOD_H
#define FLOOD_H
#pragma once


#include "../API/API.h"
// #include "..\src\lib\motors.h"
#include<stack>
#include<algorithm>
//#include <Preferences.h>



//// variable declarations
const int N = 16; 

#define memory_button 15
#define memory_switch 20

// global array for the maze, tells the robot how to move
extern char maze[N][N];


struct configuration {
    int x;
    int y;
    char dir;
};

// How to make a global variable
// https://edisciplinas.usp.br/pluginfile.php/5453726/mod_resource/content/0/Extern%20Global%20Variable.pdf

// How to use the stl stack
// https://cplusplus.com/reference/stack/stack/
extern std::stack<configuration> cellStack;
extern configuration currentCfg; // global struct for keeping track of current pos/orientation
extern configuration poppedCfg; // global struct for popped cell cause why not

extern std::stack<configuration> pathTaken;

struct openCells {
  bool openN : 1; 
  bool openS : 1;
  bool openE : 1;
  bool openW : 1;
  // Constructor that initializes each variable to be open (1)
  openCells() : openN(1), openS(1), openE(1), openW(1) {}
};
// list of walls for recursive cell update to use
extern openCells walls[N][N];



//// function declarations
void initialize();

void saveMazeToEEPROM(char maze[N][N]);
void loadMazeFromEEPROM(char maze[N][N]);
void saveWallsToEEPROM(openCells walls[N][N]);
void loadWallsFromEEPROM(openCells walls[N][N]);


void flowElevation();
openCells checkOpenCells(configuration currentCfg);
void checkNeigboringOpen(configuration poppedCfg);
void move(char direction);
void invertMaze(char goal);
void mazePrintout();
void runMaze(char goal);
void backTrack();

#endif