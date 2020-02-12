#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Chrono.h>
#include <Encoder.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <VL53L0X.h>
#include "Wire.h"
#include "graphics.h"
#include "teensy_config.h"

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

//#define DEBUG

Servo motorLeft;
Servo motorRight;

struct Speeds {
    float left;
    float right;
} commandMotorSpeeds, actualMotorSpeeds, targetMotorSpeeds, requestedMotorSpeeds;
long lastLoopTime = millis();
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

float minMagnitude(float x, float y, float z) {
    //function to find the smallest (closest to zero) value
    // specifically, find the motor that is spinning slowest
    // which is assumed to be the most representative of robot speed
    float currentMin = x;
    float currentMinMagnitude = abs(x);
    if (abs(y) < currentMinMagnitude) {
        currentMin = y;
        currentMinMagnitude = abs(y);
    }
    if (abs(z) < currentMinMagnitude) {
        currentMin = z;
    }
    return currentMin;
}

void setup() {
    // Initialise I2C bus
    Wire.begin();
    // Setup serial comms
    // Show debug warning if debug flag is set

    Serial2.begin(1152000);
    while (!Serial2) {
    };

    // Attach motors
    motorLeft.attach(TEENSY_PIN_DRIVE_LEFT);
    motorRight.attach(TEENSY_PIN_DRIVE_RIGHT);

    myTransfer.begin(Serial2);
    requestedMotorSpeeds.left = 0;
    requestedMotorSpeeds.right = 0;

    //initialise oldEncoderReadings so first loop has valid values
    for (uint8_t n = 0; n < NUM_ENCODERS; n++) {
        oldEncoderReadings[n]=0;
    }

    // Initialise ToF sensors
    tcaselect(0);
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);

        activeToFSensors[t] = sensor.init();

        if (activeToFSensors[t]) {
            sensor.setMeasurementTimingBudget(20000);
            // Start continuous back-to-back mode (take readings as
            // fast as possible).  To use continuous timed mode
            // instead, provide a desired inter-measurement period in
            // ms (e.g. sensor.startContinuous(100)).
            sensor.startContinuous();
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
            myTransfer.rxObj(requestedMotorSpeeds, sizeof(requestedMotorSpeeds), recSize);
        } else {
            missedMotorMessageCount++;
        }
    }
    // Have we missed 5 valid motor messages?
    if (missedMotorMessageCount >= 10) {
        requestedMotorSpeeds.left = 0;
        requestedMotorSpeeds.right = 0;
    }

#ifdef DEBUG
    Serial.print(requestedMotorSpeeds.left);
    Serial.print(' ');
    Serial.print(requestedMotorSpeeds.right);
    Serial.println();
#endif
    //convert -100 - +100 percentage speed command into mm/sec
    // for autonomous control we could revert back to using full scale
    // but for manual control, and for testing speedcontrol precision
    // better to start with limiting to lower speeds 
    float maxspeed_mm_per_sec = 1000;  //max acheivable is 8000
    targetMotorSpeeds.right = -requestedMotorSpeeds.right * maxspeed_mm_per_sec / 100;
    targetMotorSpeeds.left = requestedMotorSpeeds.left * maxspeed_mm_per_sec / 100;

    //convert speed commands into predicted power
    //otherwise known as feedforward. We can do feedforward
    // and/or PID speed control. both is better but either
    // alone should give functional results currently neither does 

    float minTurnPower = 18;  //determined from practical testing
    float minForwardPower = 8;  //same
    float powerCoefficient = 113;  //same
    float turnThreshold = 100;  //units: mm/sec. arbitary, value. 
    // using the turnThreshold does create a discontinuity when transitioning
    // from mostly straight ahead to a slight turn but then the two moves
    // do need different power outputs. maybe linear interpolation between
    // the two would be better?  

   // since there's a min power needed to move (as defined above)
   // first check if we're trying to move  
    if (targetMotorSpeeds.left != 0 and targetMotorSpeeds.right != 0) {
        //then check if we're trying to turn or not, i.e. left and right speeds different
        if (abs(targetMotorSpeeds.right - targetMotorSpeeds.left) > turnThreshold) {
            //then predict power needed to acheive that speed. formule dervied from curve fitting experimental results
            float turnComponent = sgn(targetMotorSpeeds.right - targetMotorSpeeds.left) * (abs(targetMotorSpeeds.right - targetMotorSpeeds.left) / powerCoefficient + minTurnPower);
            float forwardComponent = (targetMotorSpeeds.right + targetMotorSpeeds.left) / 2 / powerCoefficient;
            commandMotorSpeeds.right = turnComponent + forwardComponent;
            commandMotorSpeeds.left = -turnComponent + forwardComponent;
        } else {
            //a different formula is best fit for going straight
            commandMotorSpeeds.right = sgn(targetMotorSpeeds.right) * abs(targetMotorSpeeds.right) / powerCoefficient + minForwardPower;
            commandMotorSpeeds.left = sgn(targetMotorSpeeds.left) * abs(targetMotorSpeeds.left) / powerCoefficient + minForwardPower;
        }
    } else {
        //if we're not trying to move, turn the motors off
        commandMotorSpeeds.right = 0;
        commandMotorSpeeds.left = 0;
    }

    // apply PID
    //or at the moment, just proportional
    //. i.e power percentage proporational
    // to difference between desired speed and current actual wheel speed
    float kp = 10 / powerCoefficient;  //ie. how much power to use for a given speed error
    float loopTime = (millis() - lastLoopTime)/1000.0;  // divide by 1000 converts to seconds.
    lastLoopTime = millis();
    float travelPerEncoderCount = 1;           //millimeters per encoder count. from testing

    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec 
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {

        motorSpeeds[n] = ((float)(encoderReadings[n] - oldEncoderReadings[n])) / loopTime * travelPerEncoderCount;
    }
    
    //most representative speed assumed to be slowest wheel
    //#0 & #1 known to be on one side of bot, #3 & #5 on the other
    //at the moment, I'm not sure which is is which though...
    actualMotorSpeeds.right = minMagnitude(motorSpeeds[3], motorSpeeds[5], motorSpeeds[5]);
    actualMotorSpeeds.left = minMagnitude(motorSpeeds[0], motorSpeeds[1], motorSpeeds[1]);

    // do actual Proportional calc.
    //speed error is target - actual.
    commandMotorSpeeds.left = kp * (targetMotorSpeeds.left - actualMotorSpeeds.left);
    commandMotorSpeeds.right = kp * (targetMotorSpeeds.right - actualMotorSpeeds.right);

    //constrain output
    commandMotorSpeeds.left =max(min(commandMotorSpeeds.left, 100), -100);
    commandMotorSpeeds.right =-max(min(commandMotorSpeeds.right, 100), -100);

    // Write motorspeeds
    motorLeft.writeMicroseconds(map(commandMotorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(commandMotorSpeeds.right * -1, -100, 100, 1000, 2000));

    if (readSensors.hasPassed(10)) {
        readSensors.restart();
        // Iterate through ToF sensors and attempt to get reading
        for (uint8_t t = 0; t < 8; t++) {
            tcaselect(t);
            if (activeToFSensors[t]) {
                distances[t] = sensor.readRangeContinuousMillimeters();
                if (sensor.timeoutOccurred()) {
                    distances[t] = 0;
                }
            } else {
                distances[t] = 0;
            }
        }
        // stash key motor speed variables in ToF variable to get into log file for speed control tuning
        distances[0] = targetMotorSpeeds.left;
        distances[1] = targetMotorSpeeds.right;
        distances[2] = motorSpeeds[0];
        distances[3] = motorSpeeds[1];
        distances[4] = motorSpeeds[3];
        distances[5] = motorSpeeds[5];
        distances[6] = commandMotorSpeeds.left;
        distances[7] = commandMotorSpeeds.right;
      
        /// Read Encoder counts
        for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
            //stash old encoder readings so we know how much it changed this loop
            oldEncoderReadings[n] = encoderReadings[n];
            encoderReadings[n] = encoders[n].read();
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