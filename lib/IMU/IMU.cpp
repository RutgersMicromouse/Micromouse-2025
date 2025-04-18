#include "IMU.h"

Adafruit_BNO055 bno;

void setupIMU() {
    // put your setup code here, to run once:
    bno = Adafruit_BNO055(55, 0x28);
  
    if (!bno.begin()) Serial.print("No BNO055 detected");
    else Serial.println("IMU is setup!");
}

double angle() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    return orientationData.orientation.x;
  
}
  