#pragma once
#include <stdint.h>

typedef struct
{
  uint8_t x;
  uint8_t y;
} point;

typedef struct{
  point location;
  uint8_t direction;
} mouse_t;

typedef struct{
  float weight;
  point parent;
  bool visited;
} cell;


void initialize_maze(uint8_t x, uint8_t y,bool reset);
void floodfill();
void setupSwitches();
void saveMaze();
bool loadMaze();
void resetMazeData();
void checkSwitches();
static void fill_maze();
static bool hasnorthwall(point p);
static bool hassouthwall(point p);
static bool haseastwall(point p);
static bool haswestwall(point p);
void reflood();
static void setwalls(mouse_t mike);
static void realturn(uint8_t direction);
void set_visited(point p);

