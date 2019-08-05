/*
  PottyBox3
  By: Kevin Abraham
*/

#include <Arduino.h>
#include <i2c_t3.h>
#include "SparkFun_VL53L1X.h"

/**
 * Configuration
 */

// Minimum signal threshold (kcps/SPAD)
#define SENSOR_CAL_SIGNAL_THRESHOLD 500

// Threshold to determine if toilet is occupied (inches)
#define SENSOR_THRESHOLD 17.5

// Length of status history to determine malfunction
#define STATUS_HIST_LEN 8

// How long to wait after occupancy before delaying turn off (milliseconds)?
#define EXHAUST_DELAY_THRESHOLD 15000

// How long to wait after leaving before turning off exhaust (milliseconds)?
#define EXHAUST_DELAY_TIME 30000

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

SFEVL53L1X *sensor1 = nullptr;  // First sensor, with shutdown pin connected
SFEVL53L1X *sensor2 = nullptr;  // Second sensor (optional)

uint32_t touch1Threshold = 0, touch2Threshold = 0;

uint8_t odorState = 0;  // 0 = exhaust off, unoccupied
                        // 1 = exhaust off, occupied
                        // 2 = exhaust on, unoccupied
                        // 3 = exhaust on, occupied

elapsedMillis odorTime = 0;  // odorState = 1: time since sitting
                             // odorState = 2: time since leaving

void setup(void) {
  // Get pins low as soon as possible
  pinMode(EXHAUST_PIN, OUTPUT);
  digitalWrite(EXHAUST_PIN, LOW);
  pinMode(FLUSH_1_PIN, OUTPUT);
  digitalWrite(FLUSH_1_PIN, LOW);
  pinMode(FLUSH_2_PIN, OUTPUT);
  digitalWrite(FLUSH_2_PIN, LOW);

  pinMode(SENSOR_1_SHUTDOWN_PIN, INPUT);

  Wire1.begin();
  Wire1.resetBus();

  Serial.begin(115200);

#ifdef DEBUG
  elapsedMillis serialTimeout;
  while (!Serial && serialTimeout < 5000) {
    SIM_SRVCOP = 0x55;
    SIM_SRVCOP = 0xAA;
  }
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
    if (Wire1.endTransmission(I2C_STOP, 10000) == 0) {
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
    delay(10);
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
      Serial.println("Done.");

      // Initialize sensor 2
      Serial.print("Initializing sensor 2...                ");
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
    sensor1 = new SFEVL53L1X(Wire1);
    sensor1->begin();
    Serial.println("Done.");

    Serial.println("Sensor initialization complete.");
    Serial.println();
  } else if (count == 2) {
    sensor1 = new SFEVL53L1X(Wire1);
    sensor1->begin();
    sensor2 = new SFEVL53L1X(Wire1, 0x54);
    sensor2->begin();
  }

  if (sensor1) {
    sensor1->setROI(4, 4);
    sensor1->setTimingBudgetInMs(500);
    delay(10);
    sensor1->startRanging();
  }
  if (sensor2) {
    sensor2->setROI(4, 4);
    sensor2->setTimingBudgetInMs(500);
    delay(10);
    sensor2->startRanging();
  }

  Serial.println("Calibrating touch sensors...");

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

uint8_t status1[STATUS_HIST_LEN] = {0, 0, 0, 0};
uint8_t status2[STATUS_HIST_LEN] = {0, 0, 0, 0};

uint16_t touch1Value, touch2Value;
bool touched1, touched2;

void loop(void) {
  SIM_SRVCOP = 0x55;
  SIM_SRVCOP = 0xAA;
  delay(500);

  /**
   * Data acquisition
   */

  if (sensor1) {
    sensor1Distance = sensor1->getDistance() * 0.0393701;
    occupied1 = sensor1Distance < SENSOR_THRESHOLD &&
                sensor1->getSignalPerSpad() > SENSOR_CAL_SIGNAL_THRESHOLD;

    bool nonZeroStatus1 = true;
    for (uint8_t i = 0; i < STATUS_HIST_LEN; ++i) {
      nonZeroStatus1 = nonZeroStatus1 && status1[i] != 0 && status1[i] != 2;
    }
    if (nonZeroStatus1) {
      Serial.println("FATAL: Multiple non-zero range statuses for sensor 1...");
      pinMode(SENSOR_1_SHUTDOWN_PIN, OUTPUT);
      digitalWrite(SENSOR_1_SHUTDOWN_PIN, LOW);
      delay(10000);
    }
    for (uint8_t i = STATUS_HIST_LEN - 1; i > 0; --i) {
      status1[i] = status1[i - 1];
    }
    status1[0] = sensor1->getRangeStatus();
  }

  if (sensor2) {
    sensor2Distance = sensor2->getDistance() * 0.0393701;
    occupied2 = sensor2Distance < SENSOR_THRESHOLD &&
                sensor2->getSignalPerSpad() > SENSOR_CAL_SIGNAL_THRESHOLD;

    bool nonZeroStatus2 = true;
    for (uint8_t i = 0; i < STATUS_HIST_LEN; ++i) {
      nonZeroStatus2 = nonZeroStatus2 && status2[i] != 0 && status2[i] != 2;
    }
    if (nonZeroStatus2) {
      Serial.println("FATAL: Multiple non-zero range statuses for sensor 2...");
      pinMode(SENSOR_1_SHUTDOWN_PIN, OUTPUT);
      digitalWrite(SENSOR_1_SHUTDOWN_PIN, LOW);
      delay(10000);
    }
    for (uint8_t i = STATUS_HIST_LEN - 1; i > 0; --i) {
      status2[i] = status2[i - 1];
    }
    status2[0] = sensor2->getRangeStatus();
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
        odorState = 3;
        odorTime = 0;
      }
      break;

    case 1:
      // Exhaust off, occupied
      odorState = 3;
      odorTime = 0;
      break;

    case 3:
      // Exhaust on, occupied
      if (!occupied1 && !occupied2) {
        if (odorTime < EXHAUST_DELAY_THRESHOLD) {
          odorState = 0;
        } else {
          odorState = 2;
          odorTime = 0;
        }
      }
      break;

    case 2:
      // Exhaust on, unoccupied
      if (occupied1 || occupied2) {
        odorState = 3;
      }
      if (odorTime > EXHAUST_DELAY_TIME) {
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
  if (sensor1) {
    Serial.print("S1: ");
    Serial.print(occupied1);
    Serial.print(" (");
    Serial.print(sensor1Distance, 0);
    Serial.print(" in, ");
    Serial.print(sensor1->getSignalPerSpad());
    Serial.print(", {");
    for (uint8_t i = 0; i < STATUS_HIST_LEN; ++i) {
      Serial.print(status1[i]);
      if (i != STATUS_HIST_LEN - 1) {
        Serial.print(", ");
      }
    }
    Serial.print("})");
  }

  if (sensor2) {
    Serial.print("\tS2: ");
    Serial.print(occupied2);
    Serial.print(" (");
    Serial.print(sensor2Distance, 0);
    Serial.print(" in, ");
    Serial.print(sensor2->getSignalPerSpad());
    Serial.print(", {");
    for (uint8_t i = 0; i < STATUS_HIST_LEN; ++i) {
      Serial.print(status2[i]);
      if (i != STATUS_HIST_LEN - 1) {
        Serial.print(", ");
      }
    }
    Serial.print("})");
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

  Serial.println();
#endif
}

// redefine the startup_early_hook which by default disables the COP
#ifdef __cplusplus
extern "C" {
#endif
void startup_early_hook() {
  // empty
}
#ifdef __cplusplus
}
#endif
