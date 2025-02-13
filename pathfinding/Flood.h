#ifndef FLOOD_H
#define FLOOD_H
#pragma once


#include "API.h"
#include<stack>
#include<queue>
#include<algorithm>

#ifdef REAL
#include <EEPROM.h>

#define memory_button 15
#define memory_switch 20

#endif



//// variable declarations
const int N = 16; 

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

// uses bitfields to specify each bool to only look at 1 bit
// instead of 4 bytes/cell
struct openCells {
    bool openN : 1; 
    bool openS : 1;
    bool openE : 1;
    bool openW : 1;
    bool visited: 1; // used for fast running
    // Constructor that initializes each variable to be open (1)
    openCells() : openN(1), openS(1), openE(1), openW(1), visited(0) {}
};
// list of walls for recursive cell update to use
extern openCells walls[N][N];



//// function declarations
void initialize();

#ifdef REAL
void saveMazeToEEPROM(char maze[N][N]);
void loadMazeFromEEPROM(char maze[N][N]);
void saveWallsToEEPROM(openCells walls[N][N]);
void loadWallsFromEEPROM(openCells walls[N][N]);
#endif
#ifdef SIM
void visualizeMaze(char maze[N][N]);
void visualizeWalls(int x, int y, openCells cell);
#endif



// Flood fill algorithm
// mouse tries to approach smaller values of the maze
// as the mouse explores, it updates the values along its path. 

//Modified Flood Fill
//https://marsuniversity.github.io/ece387/FloodFill.pdf
void flowElevation();
openCells checkOpenCells(configuration currentCfg);
void checkNeigboringOpen(configuration poppedCfg);
void move(char direction);
//void invertMaze(char goal);
void mazePrintout();
void runMaze(char goal);
void backTrack();





// DONE
// converts maze and wall configuration to continuous straight and diagonals
#include<cmath>
#include<queue>
#include<vector>
#include<set>
#include<cstdint>
#include<map>

// A* algorithm
void speedrun();


struct Node
{
    int X; // node's X
    int Y; // node's Y
    int parentX; // parent node's X
    int parentY; // parent node's Y
    float gCost; // +0 cost if moving in straight line from parent's parent, +1 cost if not
    float hCost; // derive from maze as avg(maze[floor(x/2)], maze[ceil(x/2)]
    float fCost; // g+h

    // Initialize with unknown parent, infinite cost
    Node() : parentX(-1), parentY(-1), gCost(1337), fCost(1337) {}

    // Overload the less-than operator for priority queue
    bool operator>(const Node& other) const {
        return fCost > other.fCost;
    }
};


static float calculateH(int x, int y) {
	// if the H is on the 16x16 square, (1,1), take it straight from the 16x16 (0,0)
	// -> e.g. (1,3) on high res corresponds to (0,1)

	// if the H is between two squares, take the average from the two adjacent
	// -> e.g. (1,2) on high res is between (0,0) and (0,1)
	
	// 16x16 to 33x33 is 2n+1, so 33x33 to 16x16 is (n-1)/2
	int x1 = floor((x-1)/2.0);
	int y1 = floor((y-1)/2.0);
	int x2 = ceil((x-1)/2.0);
	int y2 = ceil((y-1)/2.0);
	
	float probeMaze1 = static_cast<float>(maze[x1][y1]); 
	float probeMaze2 = static_cast<float>(maze[x2][y2]);


	float H = (probeMaze1 + probeMaze2) / 2.0; 
		
	return H;

}

#endif
