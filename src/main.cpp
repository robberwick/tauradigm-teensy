#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Chrono.h>
#include <Encoder.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <utility/imumaths.h>
#include "Wire.h"
#include "graphics.h"
#include "teensy_config.h"

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define DEBUG

Servo motorLeft;
Servo motorRight;

struct MotorSpeeds {
    float left;
    float right;
} motorSpeeds;
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

Adafruit_SSD1306 display(128, 64);

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDR);

void tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void haltAndCatchFire() {
    while (1) {
    }
}

void do_i2c_scan() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("I2c Devices");
    for (uint8_t addr = 1; addr <= 127; addr++) {
        uint8_t data;
        if (!twi_writeTo(addr, &data, 0, 1, 1)) {
            display.print("0x");
            display.println(addr, HEX);
        }
    }
    display.display();
}

void printEvent(sensors_event_t* event) {
    Serial.println();
    Serial.print(event->type);
    double x = -1000000, y = -1000000, z = -1000000;  //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_ORIENTATION) {
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    } else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }

    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
}

void setup() {
    // Initialise I2C bus
    Wire.begin();

    // Initalise display and show logo
    if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) {
        // TODO show failure message on OLED
        haltAndCatchFire();
    }
    display.setTextSize(1);                              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);  // Draw white text
    display.setCursor(0, 0);                             // Start at top-left corner
    display.cp437(true);                                 // Use full 256 char 'Code Page 437' font

    display.clearDisplay();

    display.drawBitmap(
        (display.width() - LOGO_WIDTH) / 2,
        (display.height() - LOGO_HEIGHT) / 2,
        logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.display();
    delay(3000);

    // Setup serial comms
    // Show debug warning if debug flag is set
#ifdef DEBUG
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("*** WARNING ***"));
    display.println("");
    display.println("Debug flag is set");
    display.println("Waiting for\nUSB serial");
    display.display();
    delay(1000);
    Serial.begin(115200);
    while (!Serial) {
    };
#endif

    Serial2.begin(1152000);
    while (!Serial2) {
    };

    // do i2c scan
    do_i2c_scan();
    delay(2000);

    // Attach motors
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Motors"));
    motorLeft.attach(TEENSY_PIN_DRIVE_LEFT);
    motorRight.attach(TEENSY_PIN_DRIVE_RIGHT);
    // Initialise motor speeds
    motorSpeeds.left = 0;
    motorSpeeds.right = 0;
    display.setCursor(0, 10);
    display.print("OK");
    display.display();
    delay(1000);

    // Initialise serial transfer
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Serial transfer"));
    display.setCursor(0, 10);
    display.print("OK");
    myTransfer.begin(Serial2);
    display.display();
    delay(1000);

    // Initialise ToF sensors
    tcaselect(0);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("ToF sensors"));
    display.setCursor(0, 10);
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        activeToFSensors[t] = sensor.init();

        if (activeToFSensors[t]) {
            display.printf("%d: OK", t);
            sensor.setMeasurementTimingBudget(20000);
            // Start continuous back-to-back mode (take readings as
            // fast as possible).  To use continuous timed mode
            // instead, provide a desired inter-measurement period in
            // ms (e.g. sensor.startContinuous(100)).
            sensor.startContinuous();
        } else {
            display.printf("%d: FAIL", t);
        }
        if (t % 2 == 1) {
            display.println("");
        } else {
            display.setCursor(64, display.getCursorY());
        }
        display.display();
        delay(1000);
    }

    // //initialise IMU
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("IMU"));
    if (!bno.begin()) {
        display.println("FAIL");
    } else {
        display.println("OK");
    }
    display.display();
    delay(1000);
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
            myTransfer.rxObj(motorSpeeds, sizeof(motorSpeeds), recSize);
        } else {
            missedMotorMessageCount++;
        }

        sensors_event_t orientationData, angVelocityData, linearAccelData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

        printEvent(&orientationData);
        printEvent(&angVelocityData);
        printEvent(&linearAccelData);

        int8_t boardTemp = bno.getTemp();
        Serial.print(F("temperature: "));
        Serial.println(boardTemp);
    }
    // Have we missed 5 valid motor messages?
    if (missedMotorMessageCount >= 10) {
        motorSpeeds.left = 0;
        motorSpeeds.right = 0;
    }

    // Write motorspeeds
    motorLeft.writeMicroseconds(map(motorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(motorSpeeds.right * -1, -100, 100, 1000, 2000));

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

        /// Read Encoder counts
        for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
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