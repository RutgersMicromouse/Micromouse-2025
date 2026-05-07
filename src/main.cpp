#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "initializeHand.h"
#include "sidedist.h"

#include "Flood.h"
#include "labyrinth.h"
#include "firefighter.h"

const int SENSOR_RIGHT_PIN = D9; 
const int SENSOR_LEFT_PIN = D10;

bool armed = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(20000); // use 400 kHz I2C
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // turns the builtin LED on (active low)

  delay(2000); // Delay to open serial monitor
  digitalWrite(LED_BUILTIN, HIGH); // turns the builtin LED off (active low)



    
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
                }
  }

  tofSetup();
  imuSetup();
  delay(50);
  motorSetup();
  sideDistSetup();

  // move to middle of starting cell from back wall

  // for (int i = 0; i < 5; i++) {
  //   API::turnHalf();
  //   delay(200);
  // }

  // // // Switch options:
  if(isFirefighter()) { // Do nothing for now
    while(1);
        // init_GPIO();
    // firefighterSetup();
    // firefighterLoop();
    return;
  } 
  if(isSpeedrun()) {
        initialize(); // load switch should also be on
    delay(100);
    speedrun();
    return;
  }
  if(isLabyrinth()) {
        labyrinthLoop();
    return;    
  }

   startUpcheck(); // waits for hand in front to start up
   imuSetup(); // re-calibrate IMU after startup check
   pidForwardSetup();

    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180);
    // pidForward(180*200);
    // delay(5000);

  // // // Default
   initialize();
   runMaze('c');
   
   /*
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   API::turnRight();
   delay(200);
   */
   /*
   API::turnLeft();
   API::turnLeft(); 
   API::turnLeft();
   API::turnLeft();
   API::turnLeft();
   API::turnLeft(); 
   API::turnLeft();
   API::turnLeft();
   API::turnLeft();
   API::turnLeft(); 
   API::turnLeft();
   API::turnLeft();
   API::turnLeft();
   API::turnLeft(); 
   API::turnLeft();
   API::turnLeft();
   API::turnLeft();
   API::turnLeft(); 
   API::turnLeft();
   API::turnLeft();
    delay(200);
    */
  //pidForward(30.48); // move forward 12 inches to avoid touching walls

}

void loop() {
  // double leftDist = front();

  // // 5. Print the results
  // Serial.println("Front Distance: " + String(leftDist) + " mm");

  
  // // A 50ms delay is much better for responsive robotics than 1000ms!
  // delay(50);

  double leftDist = getLeftSideDist();
  double rightDist = getRightSideDist();
  // 5. Print the results
  Serial.println("Left Distance: " + String(leftDist) + " mm, Right Distance: " + String(rightDist) + " mm");
  delay(50);
  
}
