#include <Wire.h>
#include <VL53L1X.h>
#include <Arduino.h>

VL53L1X sensor;
int pinOne = 5;
int pinTwo = 4;
int16_t distanceOne, distanceTwo;

void setup() {
  // put your setup code here, to run once:
  pinMode(pinOne, INPUT);
  pinMode(pinTwo, INPUT);

  Serial.begin(115200);
  sensor.setTimeout(500);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
  // Serial.println("Hello!");

}


void loop() {
  // put your main code here, to run repeatedly:
  int16_t t1 = pulseIn(pinOne, HIGH);
  int16_t t2 = pulseIn(pinTwo, HIGH);
  distanceOne = (t1 - 1000) * 3 / 4;
  distanceTwo = (t2 - 1000) * 3 / 4;

  Serial.printf("left: %d, right: %d, center %d\n",distanceOne,distanceTwo,sensor.readRangeContinuousMillimeters());
  // if(distanceOne < 100) {
  //   Serial.println("Left Wall Detected"); 
  // } 

  // if(distanceTwo < 100) {
  //   Serial.println("Right Wall Detect");
  // } 

  delay(100);


}