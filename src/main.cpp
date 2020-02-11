#include <Arduino.h>
#include <Chrono.h>
#include <Encoder.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <VL53L0X.h>
#include "Wire.h"
#include "teensy_config.h"

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

// #define DEBUG

Servo motorLeft;
Servo motorRight;

struct Speeds {
    float left;
    float right;
} CommandMotorSpeeds, ActualMotorSpeeds, TargetMotorSpeeds;
float lastLoopTime = millis();
uint32_t missedMotorMessageCount = 0;

SerialTransfer myTransfer;

int8_t step = 1;

Chrono sendMessage;
Chrono readSensors;
VL53L0X sensor;

float distances[8];
bool activeToFSensors[8];
Encoder encoders[NUM_ENCODERS] = {
    Encoder(TEENSY_PIN_ENC1A, TEENSY_PIN_ENC1B),
    Encoder(TEENSY_PIN_ENC2A, TEENSY_PIN_ENC2B),
    Encoder(TEENSY_PIN_ENC3A, TEENSY_PIN_ENC3B),
    Encoder(TEENSY_PIN_ENC4A, TEENSY_PIN_ENC4B),
    Encoder(TEENSY_PIN_ENC5A, TEENSY_PIN_ENC5B),
    Encoder(TEENSY_PIN_ENC6A, TEENSY_PIN_ENC6B)};

long encoderReadings[NUM_ENCODERS];
long oldEncoderReadings[NUM_ENCODERS];
float motorSpeeds[NUM_ENCODERS];

void tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
int i = (b) ? 0 : 1; // assign 0 to i if b is true, otherwise 1
#define minmagnitude(x,y,z) {
    float currentMin = x;
    float currentMinMagnitude = abs(x);
    if (abs(y) < currentMinMagnitude) {
        currentMin = y;
        currentMinMagnitude = abs(y)
    }
    if (abs(z) < currentMinMagnitude) {
        currentMin = z;
    }
    return currentMin;
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

    motorLeft.attach(TEENSY_PIN_DRIVE_LEFT);
    motorRight.attach(TEENSY_PIN_DRIVE_RIGHT);

    myTransfer.begin(Serial2);
    TargetMotorSpeeds.left = 0;
    TargetMotorSpeeds.right = 0;

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
            sensor.setMeasurementTimingBudget(20000);
            sensor.startContinuous();
#ifdef DEBUG
            Serial.printf("Sensor %d init success", t);
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
    // If the message sending timeout has passed then attempt to read
    // motor speeds and apply them
    if (sendMessage.hasPassed(20)) {
        // restart the timeout
        sendMessage.restart();
        if (myTransfer.available()) {
            // reset missing motor message count
            missedMotorMessageCount = 0;
            uint8_t recSize = 0;
            myTransfer.rxObj(TargetMotorSpeeds, sizeof(TargetMotorSpeeds), recSize);
        } else {
            missedMotorMessageCount++;
        }
    }
    // Have we missed 5 valid motor messages?
    if (missedMotorMessageCount >= 10) {
        TargetMotorSpeeds.left = 0;
        TargetMotorSpeeds.right = 0;
    }
#ifdef DEBUG
    Serial.print(TargetMotorSpeeds.left);
    Serial.print(' ');
    Serial.print(TargetMotorSpeeds.right);
    Serial.println();
#endif
    //convert -100 - +100 percentage speed command into mm/sec
    float maxspeed_mm_per_sec = 1000; //max acheivable is 8000
    TargetMotorSpeeds.right = TargetMotorSpeeds.right * maxspeed_mm_per_sec/100;
    TargetMotorSpeeds.left = TargetMotorSpeeds.left * maxspeed_mm_per_sec/100;

    //convert speed commands into predicted power
    float minTurnPower = 18;
    float minForwardPower = 8;
    float powerCoefficient = 113;
    float turnThreshold = 100;
    if (TargetMotorSpeeds.left!=0 and TargetMotorSpeeds.right!=0){
        if (abs(TargetMotorSpeeds.right-TargetMotorSpeeds.left)>turnThreshold) {
            float turnComponent = sgn(TargetMotorSpeeds.right-TargetMotorSpeeds.left)*(abs(TargetMotorSpeeds.right-TargetMotorSpeeds.left)/powerCoefficient+minTurnPower);
            float forwardComponent = (TargetMotorSpeeds.right+TargetMotorSpeeds.left)/2/powerCoefficient;
            CommandMotorSpeeds.right = turnComponent + forwardComponent;
            CommandMotorSpeeds.left = -turnComponent + forwardComponent;
        } else {
            CommandMotorSpeeds.right = sgn(TargetMotorSpeeds.right)*abs(TargetMotorSpeeds.right)/powerCoefficient+minForwardPower;
            CommandMotorSpeeds.left =sgn(TargetMotorSpeeds.left)*abs(TargetMotorSpeeds.left)/powerCoefficient+minForwardPower;
        }
    } else {
        CommandMotorSpeeds.right = 0;
        CommandMotorSpeeds.left = 0;
    }
    // apply PID
    float kp = 0.1;
    float loopTime = millis()-lastLoopTime;
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        motorSpeeds[n] = (encoderReadings[n]-oldEncoderReadings[n])/loopTime;
    }
    ActualMotorSpeeds.left = min(motorSpeeds[0],motorSpeeds[1]);
    ActualMotorSpeeds.right = min(motorSpeeds[3],motorSpeeds[5]);


    // Write motorspeeds
    motorLeft.writeMicroseconds(map(CommandMotorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(CommandMotorSpeeds.right * -1, -100, 100, 1000, 2000));

    if (readSensors.hasPassed(10)) {
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

        /// Read Encoder counts

        for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
            oldEncoderReadings[n] = encoderReadings[n];
            encoderReadings[n] = encoders[n].read();
            lastLoopTime = millis();
        }

        uint16_t payloadSize = 0;

        // Prepare the distance data
        myTransfer.txObj(distances, sizeof(distances), payloadSize);
        payloadSize += sizeof(distances);

        //Prepare encoder data
        myTransfer.txObj(encoderReadings, sizeof(encoderReadings), payloadSize);
        payloadSize += sizeof(encoderReadings);

        // Send data
        myTransfer.sendData(payloadSize);
    }
}