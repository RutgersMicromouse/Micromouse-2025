#include "frontdist.h"

VL53L1X frontToF;

void tofSetup() {
    
    frontToF.setTimeout(500);
    if (!frontToF.init())
    {
        while (1);
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    frontToF.setDistanceMode(VL53L1X::Long);
    frontToF.setMeasurementTimingBudget(50000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    frontToF.startContinuous(50);
}

// distance in mm
int16_t front() {
    return frontToF.read();
}
