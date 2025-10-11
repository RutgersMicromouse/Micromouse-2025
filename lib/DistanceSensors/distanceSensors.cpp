#include "distanceSensors.h"

VL53L1X sensor;

void setupDistanceSensors() {
  delay(100);  // Wait for sensor to power up
  sensor.setBus(&Wire);
  sensor.setTimeout(500);
  
  if (!sensor.init()) {
    Serial.println("❌ Failed to detect and initialize VL53L1X sensor!");
    while (1); // Stop if sensor not found
  }

  Serial.println("✅ VL53L1X sensor initialized.");
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(15000);
  sensor.startContinuous(10);

    pinMode(rightSensor, INPUT);
    pinMode(leftSensor, INPUT);
}

boolean checkLeftWall() {

    int16_t leftTime = pulseIn(leftSensor, HIGH);
    int16_t leftDistance = (leftTime - 1000) * 3 / 4;

    if(leftDistance < 80) {
        return true;
    }  
    return false;

}

boolean checkRightWall() {
    
    int16_t rightTime = pulseIn(rightSensor, HIGH);
    int16_t rightDistance = (rightTime - 1000) * 3 / 4;
    
    if(rightDistance < 80) {
        return true;
    } 

    return false;
}

//Change front disatnce to <110 for checking front wall
int16_t checkFrontWall() {
    return sensor.read();
}

int16_t wallBrake() {
    return sensor.readRangeContinuousMillimeters();
}