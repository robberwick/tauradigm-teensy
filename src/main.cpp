#include <Arduino.h>
#include <Chrono.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include "teensy_pins.h"

// #define DEBUG

Servo motorLeft;
Servo motorRight;

struct MotorSpeeds {
    float left;
    float right;
} motorSpeeds;

SerialTransfer myTransfer;

int8_t step = 1;

Chrono sendMessage;

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
    motorSpeeds.left = 0;
    motorSpeeds.right = 0;
}

void loop() {
    // Serial.println("waiting for data");
    if (sendMessage.hasPassed(20)) {
        // restart the timeout
        sendMessage.restart();
        if (myTransfer.available()) {
            uint8_t recSize = 0;
            myTransfer.rxObj(motorSpeeds, sizeof(motorSpeeds), recSize);
#ifdef DEBUG
            Serial.print(motorSpeeds.left);
            Serial.print(' ');
            Serial.print(motorSpeeds.right);
            Serial.println();
#endif

            motorLeft.writeMicroseconds(map(motorSpeeds.left, -100, 100, 1000, 2000));
            motorRight.writeMicroseconds(map(motorSpeeds.right * -1, -100, 100, 1000, 2000));
        }
        // else if (myTransfer.status < 0)
        // {
        //   Serial.print("ERROR: ");
        //   Serial.println(myTransfer.status);
        // }
        // else
        // {
        //   Serial.print("waiting:");
        //   Serial.println(myTransfer.status);
        // }
    }
}