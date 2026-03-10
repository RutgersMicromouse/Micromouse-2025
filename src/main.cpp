#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>       // Required to fix the Adafruit BusIO compilation error
#include <Motoron.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// --- SCHEMATIC PIN DEFINITIONS ---
#define SDA_PIN 5      // PA8 / D4
#define SCL_PIN 6      // PA9 / D5
#define LEFT_SENSE A2  // PA10 / D2
#define RIGHT_SENSE A3 // PA11 / D3

// Global Instances
MotoronI2C motors(16); 
VL53L1X tof;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize I2C with XIAO-specific pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("\n--- SCHEMATIC HARDWARE CHECK ---");

  // 1. Initialize Motoron
  motors.reinitialize();
  motors.clearResetFlag();
  if (motors.getProtocolErrorFlag()) {
    Serial.println("❌ Motoron Error: Check 12V supply to VIN.");
  } else {
    Serial.println("✅ Motoron Online.");
    motors.setMaxAcceleration(1, 100);
    motors.setMaxAcceleration(2, 100);
  }

  // 2. Initialize TOF
  if (!tof.init()) {
    Serial.println("❌ TOF Error: Check 3.3V and I2C lines.");
  } else {
    Serial.println("✅ TOF Online.");
    tof.startContinuous(50);
  }

  // 3. Initialize IMU
  if (!bno.begin()) {
    Serial.println("❌ BNO055 Error: Check ADR pin or wiring.");
  } else {
    Serial.println("✅ IMU Online.");
  }

  // 4. Configure Analog Side Sensors
  analogReadResolution(12);
  
  Serial.println("Starting test in 2 seconds...");
  delay(2000);
}

void loop() {
  // Test Motors
  motors.setSpeed(1, 300);
  motors.setSpeed(2, 300);

  // Read Sensors
  int leftWall = analogRead(LEFT_SENSE);
  int rightWall = analogRead(RIGHT_SENSE);
  uint16_t frontDist = tof.read();
  
  // Read IMU
  sensors_event_t event;
  bno.getEvent(&event);

  // Diagnostics Output
  Serial.print("Wall L/R: "); Serial.print(leftWall); Serial.print("/"); Serial.print(rightWall);
  Serial.print(" | Front: "); Serial.print(frontDist);
  Serial.print(" | Heading: "); Serial.println(event.orientation.x);

  delay(100);
}