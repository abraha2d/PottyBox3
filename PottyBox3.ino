/*
  PottyBox3
  By: Kevin Abraham
*/

#include <i2c_t3.h>
#include "SparkFun_VL53L1X.h"

#define SENSOR_1_SHUTDOWN_PIN 2

SFEVL53L1X sensor1(Wire1);  // First sensor, with shutdown pin connected
SFEVL53L1X* sensor2 = nullptr;  // Second sensor (optional)

bool sensor2Present = false;

void setup(void)
{

  Wire1.begin();

  Serial.begin(115200);
  Serial.println();
  Serial.println("┌──────────────────┐");
  Serial.println("│     PottyBox3    │");
  Serial.println("│ By Kevin Abraham │");
  Serial.println("└──────────────────┘");
  Serial.println();
  Serial.println("Get the latest version at:");
  Serial.println("https://github.com/abraha2d/PottyBox3");
  Serial.println();

  Serial.println("Scanning for sensors...");
  byte count = 0;
  for (byte i = 0x08; i < 0x78; i++) {
    Wire1.beginTransmission(i);
    if (Wire1.endTransmission () == 0) {
      Serial.print("Found sensor at address 0x");
      Serial.println(i, HEX);
      count++;
      delay(1);
    }
  }
  Serial.print("Done. Found ");
  Serial.print(count, DEC);
  Serial.println(" sensor(s).");
  Serial.println();

  if (count == 1) {

    // Shut down sensor 1
    Serial.print("Shutting down sensor 1...               ");
    pinMode(SENSOR_1_SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SENSOR_1_SHUTDOWN_PIN, LOW);
    Serial.println("Done.");

    // Initiate scan for more sensors
    Serial.print("Looking for sensor 2...                 ");
    count = 0;
    for (byte i = 0x08; i < 0x78; i++) {
      Wire1.beginTransmission(i);
      if (Wire1.endTransmission () == 0) {
        count++;
        delay(1);
      }
    }

    if (count == 1) {
      sensor2 = new SFEVL53L1X(Wire1);
      sensor2Present = true;
      Serial.println("Done.");

      // Initialize sensor 2
      Serial.print("Initializing sensor 2...                ");
      while(!sensor2->checkBootState());
      sensor2->begin();
      Serial.println("Done.");

      // Change sensor 2's I2C address
      Serial.print("Assigning address 0x2A to sensor 2...   ");
      sensor2->setI2CAddress(0x54);
      Serial.println("Done.");

    } else {
      Serial.println("Fail.");
    }

    // Bring sensor 1 online
    Serial.print("Bringing sensor 1 back online...        ");
    pinMode(SENSOR_1_SHUTDOWN_PIN, INPUT);
    Serial.println("Done.");

    // Initialize sensor 1
    Serial.print("Initializing sensor 1...                ");
    while(!sensor1.checkBootState());
    sensor1.begin();
    Serial.println("Done.");

    Serial.println("Sensor initialization complete.");
    Serial.println();

  } else if (count == 2) {
    sensor2 = new SFEVL53L1X(Wire1, 0x54);
    sensor2Present = true;
  }

  sensor1.setROI(11, 11);
  sensor1.startRanging();
  if (sensor2) {
    sensor2->setROI(11, 11);
    sensor2->startRanging();
  }

}

void loop(void)
{
  Serial.print("'");
  Serial.print(sensor1.getRangeStatus());
  Serial.print("' Distance (in): ");
  Serial.print(sensor1.getDistance() * 0.0393701, 2);

  if (sensor2) {
    Serial.print("\t'");
    Serial.print(sensor2->getRangeStatus());
    Serial.print("' Distance (in): ");
    Serial.print(sensor2->getDistance() * 0.0393701, 2);
  }

  Serial.println();
  delay(100);
}
