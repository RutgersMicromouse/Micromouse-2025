#pragma once

#include <Arduino.h>

bool CHECK_BEACON(void);
void IR_triggerLatch();

void init_GPIO();
void firefighterSetup();
void firefighterLoop();
