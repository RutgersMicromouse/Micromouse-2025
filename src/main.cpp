#include <Wire.h>
#include <Arduino.h>
// #include "../lib/IMU/imu.h"
// #include "../lib/distanceSensor/distance_sensor.h"
// #include "../lib/Motors/motors.h"
// #include "../lib/pidRotate/pidRotate.h"
// #include "../lib/pidStraight/pidStraight.h"
// #include "../pathfinding/API/API.h"
// #include "../pathfinding/Flood/Flood.h"
// #include "../pathfinding/simAPI/simAPI.h"
// #include "../pathfinding/Labyrinth/labyrinth.h"


//VL53L1X sensor;

int pinOne = 5;
int pinTwo = 4;
int16_t distanceOne, distanceTwo;

void setup()
{

//   imuSetup();
//   sensorInit();
//   motorSetup();
//   delay(500);
// //  pinMode(LED_BUILTIN, OUTPUT);
//   delay(500);
//   // digitalWrite(LED_BUILTIN, HIGH);
//   setForwardPWM(50);
//   delay(500);
//   initialize();
//   Serial.println("Hello world");


  pinMode(pinOne, INPUT);
  pinMode(pinTwo, INPUT);

 Serial.begin(115200);
 Serial.println("Hello!");

}

void loop()
{

  int16_t t1 = pulseIn(pinOne, HIGH);
  int16_t t2 = pulseIn(pinTwo, HIGH);

  distanceOne = (t1 - 1000) * 3 / 4;
  distanceTwo = (t2 - 1000) * 3 / 4;

  if(distanceOne < 100) {
    Serial.println("Left Wall Detected"); 
  } 

  if(distanceTwo < 100) {
     Serial.println("Right Wall Detect");
  } else {
    Serial.println("Right wall not detected");
  }

 // delay(100);


}