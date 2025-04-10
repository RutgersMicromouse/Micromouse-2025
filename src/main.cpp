#include <Wire.h>
#include <VL53L1X.h>
#include<Adafruit_BNO055.h>
// Create a custom I2C bus on pins 21 (SDA) and 22 (SCL)
VL53L1X sensor;
Adafruit_BNO055 franny;
void setup()
{
  Serial.begin(115200);
  Wire.begin(1,2);
  sensor.setAddress(0x29); // Optional: only needed if you've changed the default
  sensor.setTimeout(500);
  sensor.setBus(&Wire);
  if (!sensor.init()) // Pass custom Wire instance
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  Serial.println("Sensor initialized");

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
}

void loop()
{
  sensor.read();

  Serial.print("range: ");
  Serial.print(sensor.ranging_data.range_mm);
  Serial.print("\tstatus: ");
  Serial.print(VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
  Serial.print("\tpeak signal: ");
  Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
  Serial.print("\tambient: ");
  Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);

  Serial.println();
}
