#include "frontdist.h"

VL53L1X frontToF;

void tofSetup() {
    
    frontToF.setTimeout(500);
    if (!frontToF.init())
    {
        while (1);
        Serial.println("Failed to detect and initialize front ToF sensor!");
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    frontToF.setDistanceMode(VL53L1X::Short);
    frontToF.setMeasurementTimingBudget(20000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    frontToF.startContinuous(20);
    Serial.println("Front ToF initialized");
}

// distance in mm
int16_t front() {
    return frontToF.read();
}
