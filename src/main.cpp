#include <Arduino.h>
#include "ioexpander.h"
#include "imu.h"
#include "frontdist.h"
#include "motors.h"
#include "pidstraight.h"
#include "pidrotate.h"
#include "initializeHand.h"

#include "Flood.h"
#include "labyrinth.h"
#include "firefighter.h"

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

  //  pidForward(180);
  //  delay(5000);

  // // Default
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
  delay(2000);
          
  
  

}
