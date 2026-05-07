#include "sidedist.h"

// Define the pins locally in this file
const int SENSOR_RIGHT_PIN = D9; 
const int SENSOR_LEFT_PIN = D10;

// Call this ONCE in your main setup()
void sideDistSetup() {
  pinMode(SENSOR_LEFT_PIN, INPUT);
  pinMode(SENSOR_RIGHT_PIN, INPUT);
}

// Function to read the left sensor
double getLeftSideDist()
{
    int16_t leftTime = pulseIn(SENSOR_LEFT_PIN, HIGH);
    double leftDistance = (leftTime - 1000) * 3 / 4;
    return leftDistance / 10;
}

double getRightSideDist()
{
    int16_t rightTime = pulseIn(SENSOR_RIGHT_PIN, HIGH);
    double rightDistance = (rightTime - 1000) * 3 / 4;
    return rightDistance / 10;
}
