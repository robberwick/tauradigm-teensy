#include <Adafruit_ADS1015.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Chrono.h>
#include <EEPROM.h>
#include <Encoder.h>
#include <SerialTransfer.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <utility/imumaths.h>

#include <unordered_map>

#include "Wire.h"
#include "config.h"
#include "robot_hal.h"
#include "screen.h"
#include "status.h"
#include "types.h"
#include "utils.h"

Status robotStatus = Status();

// #define DEBUG

#ifndef ARDUINO_TEENSY31
HardwareSerial Serial2(USART2);
#endif

RobotHal hal(robotStatus);

SerialTransfer myTransfer;

Chrono receiveMessage;
Chrono readSensors;
VL53L0X sensor;

bool activeToFSensors[8];
int16_t lightSensors[4];

Encoder encoders[NUM_ENCODERS] = {
    Encoder(TEENSY_PIN_ENC1A, TEENSY_PIN_ENC1B),
    Encoder(TEENSY_PIN_ENC2A, TEENSY_PIN_ENC2B),
    Encoder(TEENSY_PIN_ENC3A, TEENSY_PIN_ENC3B),
    Encoder(TEENSY_PIN_ENC4A, TEENSY_PIN_ENC4B),
    Encoder(TEENSY_PIN_ENC5A, TEENSY_PIN_ENC5B),
    Encoder(TEENSY_PIN_ENC6A, TEENSY_PIN_ENC6B)};

Screen screen(robotStatus, 128, 64);

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDR);

void tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }

    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void haltAndCatchFire() {
    while (1) {
    }
}

void do_i2c_scan() {
    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println("I2c Devices");
    for (uint8_t addr = 1; addr <= 127; addr++) {
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission() == 0) {
            //C++20 has a contains() method for unordered_map
            // but find() is only one available to us?
            if (I2C_ADDRESS_NAMES.find(addr) != I2C_ADDRESS_NAMES.end()) {
                screen.display.println(I2C_ADDRESS_NAMES.at(addr));
            } else {
                screen.display.print("0x");
                screen.display.println(addr, HEX);
            }
        }
    }
    screen.display.display();
    delay(4000);
}

void processMessage(SerialTransfer &transfer) {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;

    // Get message type, indicated by the first byte of the message
    uint8_t messageType;
    recSize = myTransfer.rxObj(messageType, recSize);
    switch (messageType) {
        // 0 - motor speed message
        case 1:
        default:
            Speeds requestedMotorSpeeds;
            transfer.rxObj(requestedMotorSpeeds, recSize);
            hal.setMotorSpeeds(requestedMotorSpeeds);
            // reset the missed motor mdessage count
            robotStatus.resetMissedMotorCount();
            // We received a valid motor command, so reset the timer
            receiveMessage.restart();
    }
}

void post() {
    // do i2c scan
    do_i2c_scan();

    // Attach motors
    // init motors
    hal.initialiseMotors();
    screen.setMode(Screen::Mode::POST_MOTORS);
    screen.showScreen();
    delay(500);

    // Initialise serial transfer
    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println(F("Serial transfer"));

    screen.display.display();
    screen.display.print("OK");
    myTransfer.begin(Serial2);
    screen.display.display();
    delay(500);

    // Initialise ToF sensors
    tcaselect(0);
    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println(F("ToF sensors"));
    screen.display.display();
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        screen.display.printf("initialising %d", t);
        screen.display.display();
        activeToFSensors[t] = sensor.init();
        screen.display.setCursor(0, screen.display.getCursorY() + 1);
        screen.display.printf("init %d done", t);

        if (activeToFSensors[t]) {
            screen.display.printf("%d: OK", t);
            sensor.setMeasurementTimingBudget(33000);
            // lower the return signal rate limit (default is 0.25 MCPS)
            sensor.setSignalRateLimit(0.1);
            // increase laser pulse periods (defaults are 14 and 10 PCLKs)
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
            sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
            // Start continuous back-to-back mode (take readings as
            // fast as possible).  To use continuous timed mode
            // instead, provide a desired inter-measurement period in
            // ms (e.g. sensor.startContinuous(100)).
            sensor.startContinuous();
        } else {
            screen.display.printf("%d: FAIL", t);
        }
        if (t % 2 == 1) {
            screen.display.println("");
        } else {
            screen.display.setCursor(64, screen.display.getCursorY());
        }
        screen.display.display();
        delay(50);
    }
    delay(500);
    screen.display.clearDisplay();
    screen.display.display();

    // //initialise IMU
    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println(F("IMU"));
    // Do we have an IMU
    if (!bno.begin()) {
        screen.display.println("FAIL");
        screen.display.display();
        delay(3000);
    } else {
        screen.display.println("OK");
        screen.display.display();
        delay(3000);

        screen.display.clearDisplay();
        screen.display.setCursor(0, 0);

        // look for calibration data. if it exists, load it.
        // if not, calibrate then store the data in the EEPROM
        int eeAddress = 0;
        long bnoID;
        EEPROM.get(eeAddress, bnoID);

        adafruit_bno055_offsets_t calibrationData;
        sensor_t sensor;

        bno.getSensor(&sensor);
        if (bnoID != sensor.sensor_id) {
            screen.display.println("No Calibration Data in EEPROM");

        } else {
            screen.display.println("Found Calibration in EEPROM.");
            eeAddress += sizeof(long);
            EEPROM.get(eeAddress, calibrationData);

            screen.display.println("Restoring...");
            bno.setSensorOffsets(calibrationData);

            screen.display.println("Restored");
        }
        screen.display.display();
        delay(1000);

        screen.display.clearDisplay();
        screen.display.setCursor(0, 0);
        screen.display.println("Move robot now to check calibration");
        screen.display.display();
        delay(5000);
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        sensors_event_t event;
        bno.getEvent(&event);
        screen.display.println("Move sensor to calibrate magnetometers");
        screen.display.display();
        u_int8_t curYPos = screen.display.getCursorY();
        while (!system) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            /* Display the individual values */
            screen.display.setCursor(0, curYPos);
            screen.display.print("Sys:");
            screen.display.print(system, DEC);
            screen.display.print(" G:");
            screen.display.print(gyro, DEC);
            screen.display.print(" A:");
            screen.display.print(accel, DEC);
            screen.display.print(" M:");
            screen.display.println(mag, DEC);
            screen.display.setCursor(0, curYPos + 10);
            /* Display the individual values */
            delay(BNO055_SAMPLERATE_DELAY_MS);
            screen.display.print("X:");
            screen.display.print(event.orientation.x, 4);
            screen.display.print(" Y:");
            screen.display.print(event.orientation.y, 4);
            screen.display.print(" Z:");
            screen.display.println(event.orientation.z, 4);
            screen.display.display();
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
        screen.display.println("calibrated OK");
        screen.display.display();
        delay(3000);

        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);
        screen.display.clearDisplay();
        screen.display.setCursor(0, 0);

        screen.display.println("Storing calibration data to EEPROM...");

        eeAddress = 0;
        bno.getSensor(&sensor);
        bnoID = sensor.sensor_id;

        EEPROM.put(eeAddress, bnoID);

        eeAddress += sizeof(long);
        EEPROM.put(eeAddress, newCalib);
        screen.display.println("Data stored to EEPROM.");
        screen.display.display();

        delay(2000);
    }

    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println("Running");
    screen.display.display();
}

void setup() {
// Initialise I2C bus
#ifdef ARDUINO_TEENSY31
    Wire.begin();
#else
    Wire.begin(TEENSY_PIN_I2C_SDA, TEENSY_PIN_I2C_SCL);
#endif

    // Initalise display
    if (!screen.initDisplay()) {
        haltAndCatchFire();
    }
    // Show Splash Screen
    screen.setMode(Screen::Mode::START_UP);
    screen.showScreen();
    delay(3000);

    // Setup serial comms
    // Show debug warning if debug flag is set
#ifdef DEBUG
    screen.display.clearDisplay();
    screen.display.setCursor(0, 0);
    screen.display.println(F("*** WARNING ***"));
    screen.display.println("");
    screen.display.println("Debug flag is set");
    screen.display.println("Waiting for\nUSB serial");
    screen.display.display();
    delay(1000);
    Serial.begin(115200);
    while (!Serial) {
    };
#endif

    Serial2.begin(1000000);
    while (!Serial2) {
    };

    screen.setMode(Screen::Mode::GIT_STATUS);
    screen.showScreen();
    delay(2000);

    screen.setMode(Screen::Mode::PRE_POST);
    screen.showScreen();
    delay(2000);

    pinMode(TEENSY_PIN_BUTTON, INPUT_PULLUP);
    uint32_t buttonThreshold = 30;  //1024 should be supply voltage, button pulls pin low
    bool enterPost = false;
    if (analogRead(TEENSY_PIN_BUTTON) < buttonThreshold) {
        enterPost = true;
        screen.display.println("Entering POST...");
        screen.display.display();
        delay(500);
    }

    if (enterPost) {
        post();
    } else {
        // Initialise motors
        hal.initialiseMotors();

        // Initialise serial transfer
        myTransfer.begin(Serial2);

        // Initialise ToF sensors
        tcaselect(0);
        for (uint8_t t = 0; t < 8; t++) {
            tcaselect(t);
            activeToFSensors[t] = sensor.init();
            if (activeToFSensors[t]) {
                sensor.setMeasurementTimingBudget(33000);
                // lower the return signal rate limit (default is 0.25 MCPS)
                sensor.setSignalRateLimit(0.1);
                // increase laser pulse periods (defaults are 14 and 10 PCLKs)
                sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
                sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
                // Start continuous back-to-back mode (take readings as
                // fast as possible).  To use continuous timed mode
                // instead, provide a desired inter-measurement period in
                // ms (e.g. sensor.startContinuous(100)).
                sensor.startContinuous();
            }
        }

        // //initialise IMU
        screen.display.clearDisplay();
        screen.display.setCursor(0, 0);
        if (!bno.begin()) {
            screen.display.println("IMU FAIL");
        } else {
            screen.display.println("IMU CAL:");
        }
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;

        u_int8_t curYPos = screen.display.getCursorY();
        while (!system) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            screen.display.setCursor(0, curYPos);
            /* Display the individual values */
            screen.display.print("Sys:");
            screen.display.print(system, DEC);
            screen.display.print(" G:");
            screen.display.print(gyro, DEC);
            screen.display.print(" A:");
            screen.display.print(accel, DEC);
            screen.display.print(" M:");
            screen.display.println(mag, DEC);
            screen.display.display();
        };
        screen.display.println("calibrated OK");
        screen.display.display();
        delay(200);
    }
}

void loop() {
    // Is there an incoming message available?
    if (myTransfer.available()) {
        processMessage(myTransfer);
    }

    // if the message sending timeout has passed then increment the missed count
    // and reset
    if (receiveMessage.hasPassed(20)) {
        robotStatus.incrementMissedMotorCount();
        receiveMessage.restart();
    }

    // screen.display.invertDisplay(shouldInvertDisplay);
    if (robotStatus.batteryIsLow() || robotStatus.motorMessageCommsDown()) {
        screen.setMode(Screen::Mode::ERROR);

        // TODO move this outside the if block when we're only using the screen class for displaying info
        screen.showScreen();

        // Set motors to dead stop
        hal.stopMotors();
    }

    if (readSensors.hasPassed(10)) {
        readSensors.restart();
        // Iterate through ToF sensors and attempt to get reading
        for (uint8_t t = 0; t < 8; t++) {
            tcaselect(t);
            if (activeToFSensors[t]) {
                robotStatus.sensors.tofDistances[t] = sensor.readRangeContinuousMillimeters();
                if (sensor.timeoutOccurred()) {
                    robotStatus.sensors.tofDistances[t] = 0;
                }
            } else {
                robotStatus.sensors.tofDistances[t] = 0;
            }
        }

        /// Read Encoder counts
        for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
            robotStatus.sensors.encoders.previous[n] = robotStatus.sensors.encoders.current[n];
            robotStatus.sensors.encoders.current[n] = encoders[n].read();
        }

        // Read IMU
        // Update orientation status
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        robotStatus.updateOrientation(orientationData);

        uint16_t payloadSize = 0;

        //update odometry
        // float distanceMoved = getDistanceTravelled();
        float distanceMoved = hal.getDistanceTravelled();
        // Update pose based on current heading and distance moved
        // ensure that the orientation data has been updated first
        // TODO tidy this up
        robotStatus.updatePose(robotStatus.orientation.x, distanceMoved);

        // Prepare the distance data
        payloadSize = myTransfer.txObj(robotStatus.sensors.tofDistances, payloadSize);

        //Prepare encoder data
        payloadSize = myTransfer.txObj(robotStatus.sensors.encoders.current, payloadSize);

        //Prepare IMU data
        payloadSize = myTransfer.txObj(robotStatus.orientation, payloadSize);

        //Prepare odometry data
        payloadSize = myTransfer.txObj(robotStatus.pose, payloadSize);

        // Send data
        myTransfer.sendData(payloadSize);
    }
}