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
double getLeftSideDist() {
  unsigned long pulse = pulseIn(SENSOR_LEFT_PIN, HIGH, 25000);
  if (pulse > 0 && pulse < 2000) {
    double dist = 0.75 * (pulse - 1000);
    return (dist < 0) ? 0 : dist; // return 0 if negative, else return dist
  }
  return -1; // Out of range or error
}

// Function to read the right sensor
double getRightSideDist() {
  unsigned long pulse = pulseIn(SENSOR_RIGHT_PIN, HIGH, 25000);
  if (pulse > 0 && pulse < 2000) {
    double dist = 0.75 * (pulse - 1000);
    return (dist < 0) ? 0 : dist; 
  }
  return -1; // Out of range or error
}