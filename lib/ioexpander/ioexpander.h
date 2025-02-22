#pragma once

#include <Arduino.h>
#include <Wire.h>

//A0, A1, A2 all grounded
// Table 5:
// https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
#define IO_expander_addr 0x38

byte ioExpanderRead();
bool isSaving();
bool isLoad();
bool isSpeedrun();
bool isLabyrinth();
bool isFirefighter();
bool leftWall();
bool rightWall();