#include "distanceSensors.h"

VL53L1X sensor;

void setupDistanceSensors()
{
  Serial.begin(115200);
  Wire.begin(1,2);
  sensor.setAddress(0x29); // Optional: only needed if you've changed the default
  sensor.setTimeout(500);
  sensor.setBus(&Wire);
  if (!sensor.init()) // Pass custom Wire instance
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  Serial.println("Sensor initialized");
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

    pinMode(rightSensor, INPUT);
    pinMode(leftSensor, INPUT);
}

boolean checkLeftWall() {

    int16_t leftTime = pulseIn(leftSensor, HIGH);
    int16_t leftDistance = (leftTime - 1000) * 3 / 4;

    if(leftDistance < 100) {
        return true;
    }  
    return false;

}

boolean checkRightWall() {
    
    int16_t rightTime = pulseIn(rightSensor, HIGH);
    int16_t rightDistance = (rightTime - 1000) * 3 / 4;
    
    if(rightDistance < 100) {
        return true;
    } 

    return false;
}

int16_t checkFrontWall() {
    return sensor.read();
}