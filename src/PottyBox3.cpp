/*
  PottyBox3
  By: Kevin Abraham
*/

#define DEBUG

#include <Arduino.h>
#include <i2c_t3.h>
#include "SparkFun_VL53L1X.h"

/**
 * Configuration
 */

// Threshold to determine if toilet is occupied (in inches)
#define SENSOR_THRESHOLD 20

// How long to wait after occupancy before turning on exhaust (in milliseconds)?
#define EXHAUST_ON_WAIT 5000

// How long to wait after de-occupancy before turning off exhaust (in milliseconds)?
#define EXHAUST_OFF_WAIT 60000

// Number of samples to establish touchRead baseline
#define TOUCH_INIT_NUM_SAMPLES 10

// How much higher should reading be above baseline to count as a touch?
#define TOUCH_1_THRESHOLD 1.05
#define TOUCH_2_THRESHOLD 1.5

/**
 * Pin definitions
 */

#define EXHAUST_PIN 21

#define FLUSH_1_PIN 16
#define FLUSH_2_PIN 17

#define SENSOR_1_SHUTDOWN_PIN 2

#define TOUCH_1_PIN 3
#define TOUCH_2_PIN 4

/**
 * Global variables
 */

SFEVL53L1X sensor1(Wire1);      // First sensor, with shutdown pin connected
SFEVL53L1X *sensor2 = nullptr;  // Second sensor (optional)

bool sensor2Present = false;

uint32_t touch1Threshold = 0, touch2Threshold = 0;

uint8_t odorState = 0;  // 0 = exhaust off, unoccupied
                        // 1 = exhaust off, occupied
                        // 2 = exhaust on, unoccupied
                        // 3 = exhaust on, occupied

elapsedMillis odorTime = 0; // odorState = 1: time since sitting
                            // odorState = 2: time since leaving

void setup(void) {
  // Get pins low as soon as possible
  pinMode(EXHAUST_PIN, OUTPUT);
  digitalWrite(EXHAUST_PIN, LOW);
  pinMode(FLUSH_1_PIN, OUTPUT);
  digitalWrite(FLUSH_1_PIN, LOW);
  pinMode(FLUSH_2_PIN, OUTPUT);
  digitalWrite(FLUSH_2_PIN, LOW);

  Wire1.begin();
  Wire1.resetBus();

  Serial.begin(115200);

#ifdef DEBUG
  while (!Serial)
    ;
#endif

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
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found sensor at address 0x");
      Serial.println(i, HEX);
      count++;
      delay(1);
    }
  }
  Serial.print("Done. Found ");
  Serial.print(count, DEC);
  if (count == 1) {
    Serial.println(" sensor.");
  } else {
    Serial.println(" sensors.");
  }
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
      if (Wire1.endTransmission() == 0) {
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
      while (!sensor2->checkBootState())
        ;
      sensor2->begin();
      Serial.println("Done.");

      // Change sensor 2's I2C address
      Serial.print("Setting sensor 2's address to 0x2A...   ");
      sensor2->setI2CAddress(0x54);
      Serial.println("Done.");
    } else {
      Serial.println("Fail.");
    }

    // Bring sensor 1 online
    Serial.print("Bringing sensor 1 back online...        ");
    pinMode(SENSOR_1_SHUTDOWN_PIN, INPUT);
    while (!sensor1.checkBootState())
      ;
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

  Serial.println("Calibration touch sensors...");

  Serial.print("Touch 1...  ");
  for (uint8_t i = 0; i < TOUCH_INIT_NUM_SAMPLES; ++i) {
    touch1Threshold += touchRead(TOUCH_1_PIN);
  }
  touch1Threshold /= TOUCH_INIT_NUM_SAMPLES;
  Serial.println("Done.");

  Serial.print("Touch 2...  ");
  for (uint8_t i = 0; i < TOUCH_INIT_NUM_SAMPLES; ++i) {
    touch2Threshold += touchRead(TOUCH_2_PIN);
  }
  touch2Threshold /= TOUCH_INIT_NUM_SAMPLES;
  Serial.println("Done.");

  Serial.print("Results: Touch 1 = ");
  Serial.print(touch1Threshold);
  Serial.print(", Touch 2 = ");
  Serial.println(touch2Threshold);
  Serial.println();

  touch1Threshold *= TOUCH_1_THRESHOLD;
  touch2Threshold *= TOUCH_2_THRESHOLD;

  Serial.println("Starting main loop...");
  Serial.println();
}

float sensor1Distance, sensor2Distance;
bool occupied1, occupied2;

uint16_t touch1Value, touch2Value;
bool touched1, touched2;

void loop(void) {
  delay(100);

  /**
   * Data acquisition
   */

  sensor1Distance = sensor1.getDistance() * 0.0393701;
  occupied1 = sensor1Distance < SENSOR_THRESHOLD;

  if (sensor2) {
    sensor2Distance = sensor2->getDistance() * 0.0393701;
    occupied2 = sensor2Distance < SENSOR_THRESHOLD;
  }

  touch1Value = touchRead(TOUCH_1_PIN);
  touched1 = touch1Value > touch1Threshold;

  touch2Value = touchRead(TOUCH_2_PIN);
  touched2 = touch2Value > touch2Threshold;

  /**
   * State machine
   */

  switch (odorState) {
    case 0:
      // Exhaust off, unoccupied
      if (occupied1 || occupied2) {
        odorState = 1;
        odorTime = 0;
      }
      break;

    case 1:
      // Exhaust off, occupied
      if (!occupied1 && !occupied2) {
        odorState = 0;
      }
      if (odorTime > EXHAUST_ON_WAIT) {
        odorState = 3;
      }
      break;

    case 3:
      // Exhaust on, occupied
      if (!occupied1 && !occupied2) {
        odorState = 2;
        odorTime = 0;
      }
      break;

    case 2:
      // Exhaust on, unoccupied
      if (occupied1 || occupied2) {
        odorState = 3;
      }
      if (odorTime > EXHAUST_OFF_WAIT) {
        odorState = 0;
      }
      break;

    default:
      Serial.print("WARNING: Unknown odorState ");
      Serial.print(odorState);
      Serial.println("! Resetting to 0...");
      odorState = 0;
      odorTime = 0;
      break;
  }

  /**
   * Output activation
   */

  digitalWrite(EXHAUST_PIN, odorState >> 1);
  digitalWrite(FLUSH_1_PIN, touched1);
  digitalWrite(FLUSH_2_PIN, touched2);

  /**
   * Debug
   */

#ifdef DEBUG
  Serial.print("S1: ");
  Serial.print(occupied1);
  Serial.print(" (");
  Serial.print(sensor1Distance, 0);
  Serial.print(" in)");

  if (sensor2) {
    Serial.print("\tS2: ");
    Serial.print(occupied2);
    Serial.print(" (");
    Serial.print(sensor2Distance, 0);
    Serial.print(" in)");
  }

  Serial.print("\tT1: ");
  Serial.print(touched1);
  Serial.print(" (");
  Serial.print(touch1Value);
  Serial.print(")");

  Serial.print("\tT2: ");
  Serial.print(touched2);
  Serial.print(" (");
  Serial.print(touch2Value);
  Serial.print(")");

  Serial.print("\tState: ");
  Serial.print(odorState);
  if (__builtin_popcount(odorState) == 1) {
    Serial.print(" (");
    Serial.print(odorTime);
    Serial.print(")");
  }


  Serial.print("\tE: ");
  Serial.print(odorState >> 1);
#endif

  Serial.println();
}
