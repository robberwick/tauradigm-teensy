#include <Arduino.h>
#include <Chrono.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <VL53L0X.h>
#include "Wire.h"
#include "teensy_config.h"
#include <Encoder.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define DEBUG
Chrono readSensors;
VL53L0X sensor;

float distances[8];
bool activeToFSensors[8];


void tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) {
    };
#endif

    Serial2.begin(1152000);
    while (!Serial2) {
    };

    Wire.begin();
    tcaselect(0);
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
#ifdef DEBUG
        Serial.print("TCA Port #");
        Serial.println(t);
        sensor.setTimeout(500);
        Serial.print("init sensor: ");
        Serial.println(t);
#endif
        // Start continuous back-to-back mode (take readings as
        // fast as possible).  To use continuous timed mode
        // instead, provide a desired inter-measurement period in
        // ms (e.g. sensor.startContinuous(100)).

        activeToFSensors[t] = sensor.init();

        if (activeToFSensors[t]) {
            // sensor.setMeasurementTimingBudget(200000);
            sensor.startContinuous();
#ifdef DEBUG
            Serial.printf("Sensor %d init success", t);
            Serial.printf("measurement timing budget: %d", sensor.getMeasurementTimingBudget());
#endif
        } else {
            /*
            TODO What to do to indicate sensor init failure?
            don't attempt to read from failed sensor? show in i2c oled?
            */
#ifdef DEBUG
            Serial.print("Failed to detect and initialize sensor: ");
            Serial.println(t);
            // while (1) {
            // }

#endif
        }
    }
}

void loop() {
    if (readSensors.hasPassed(100)) {
        readSensors.restart();
    // Iterate through ToF sensors and attempt to get reading
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        if (activeToFSensors[t]) {
            distances[t] = sensor.readRangeContinuousMillimeters();
            if (sensor.timeoutOccurred()) {
                distances[t] = 0;
#ifdef DEBUG
                Serial.printf("TIMEOUT READING ToF %d", t);
#endif
            }
        } else {
            distances[t] = 0;
        }
    }
#ifdef DEBUG
    Serial.printf(
        "distances: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
        distances[0],
        distances[1],
        distances[2],
        distances[3],
        distances[4],
        distances[5],
        distances[6],
        distances[7]);
    Serial.println();
#endif
    }
}