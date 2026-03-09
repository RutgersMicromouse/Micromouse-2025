#include "distanceSensors.h"

VL53L1X frontSensor;

void setupDistanceSensors() {
    delay(100);

    frontSensor.setBus(&Wire);
    frontSensor.setTimeout(500);

    if (!frontSensor.init()) {
        Serial.println("Failed to detect and initialize VL53L1X sensor!");
        while (1);
    }

    frontSensor.setDistanceMode(VL53L1X::Long);
    frontSensor.setMeasurementTimingBudget(50000);
    frontSensor.startContinuous(50);

    Serial.println("Front distance sensor initialized.");
}

// Returns distance to front wall in mm
int16_t checkFrontDist() {
    return frontSensor.read();
}
