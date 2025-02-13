#pragma once
#include "Wire.h"
#include <Arduino.h>



byte io_expander_read();
bool is_saving();
bool is_load();
bool is_speedmode();
bool is_labyrinth();
bool is_firefighter();
bool left_wall();
bool right_wall();