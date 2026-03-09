#include "shared.h"

volatile double   g_angle          = 0.0;
volatile int16_t  g_frontDist      = 9999;
volatile bool     g_leftWall       = false;
volatile bool     g_rightWall      = false;
double            g_calibratedNorth = 0.0;
SemaphoreHandle_t i2cMutex         = NULL;
