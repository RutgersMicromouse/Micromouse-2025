#pragma once
#include <stdint.h>
#include<queue>
#include "API.h"

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

