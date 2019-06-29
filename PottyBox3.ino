/*
  PottyBox3
  By: Kevin Abraham
*/

#include <i2c_t3.h>
#include "SparkFun_VL53L1X.h"

#define SENSOR_1_SHUTDOWN_PIN 2

SFEVL53L1X distanceSensor1(Wire1);  // First sensor, with shutdown pin connected
SFEVL53L1X distanceSensor2(Wire1);  // Second sensor

void setup(void)
{

  pinMode(SENSOR_1_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SENSOR_1_SHUTDOWN_PIN, LOW);

  Wire1.begin();

  Serial.begin(115200);
  Serial.println();
  Serial.println("┌──────────────────┐");
  Serial.println("│     PottyBox3    │");
  Serial.println("│ By Kevin Abraham │");
  Serial.println("└──────────────────┘");
  Serial.println();
  Serial.println("Get latest version at:");
  Serial.println("https://github.com/abraha2d/PottyBox3");
  Serial.println();

  // Change sensor 2's I2C address
  while(!distanceSensor2.checkBootState());
  distanceSensor2.begin();
  distanceSensor2.setI2CAddress(0x54);

  // Bring sensor 1 online
  pinMode(SENSOR_1_SHUTDOWN_PIN, INPUT);
  while(!distanceSensor1.checkBootState());
  distanceSensor1.begin();

  Serial.println ("I2C scanner. Scanning...");
  byte count = 0;
  for (byte i = 1; i < 120; i++) {
    Wire1.beginTransmission(i);
    if (Wire1.endTransmission () == 0) {
      Serial.print("Found address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);
    }
  }
  Serial.print("Done. Found ");
  Serial.print(count, DEC);
  Serial.println(" device(s).");
  Serial.println();

}

void loop(void)
{
  distanceSensor1.startRanging();
  int distance1 = distanceSensor1.getDistance();
  distanceSensor1.stopRanging();

  Serial.print("\tDistance (in): ");
  Serial.print(distance1 * 0.0393701, 2);

  distanceSensor2.startRanging();
  int distance2 = distanceSensor2.getDistance();
  distanceSensor2.stopRanging();

  Serial.print("\tDistance (in): ");
  Serial.print(distance2 * 0.0393701, 2);

  Serial.println();
  delay(100);
}
