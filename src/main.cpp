#include <Adafruit_ADS1015.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Chrono.h>
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
#include "waypoint_navigation.h"
#include "routes.h"


Status robotStatus = Status();

// #define DEBUG

#ifndef ARDUINO_TEENSY31
HardwareSerial Serial2(USART2);
#endif

RobotHal hal(robotStatus);
WaypointNav navigation(route, sizeof(route)/sizeof(Pose));
controlMode state = RC;

SerialTransfer myTransfer;

// timers
Chrono receiveMessageTimeout;
Chrono sendSensorDataTimeout;
Chrono updateIMUTimeout;
Chrono updateTOFTimeout;
Chrono updateEncodersTimeout;
Chrono challengeTimeout;

int16_t lightSensors[4];

Screen screen(robotStatus, 128, 64);

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDR);

void haltAndCatchFire() {
    while (1) {
    }
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
            {
            Speeds requestedMotorSpeeds;
            transfer.rxObj(requestedMotorSpeeds, recSize);
            if (state == RC){
                hal.setRequestedMotorSpeeds(requestedMotorSpeeds);
            }
            // reset the missed motor mdessage count
            robotStatus.resetMissedMotorCount();
            // We received a valid motor command, so reset the timer
            receiveMessageTimeout.restart();
            break;
            }
        case 2:
            char button;
            transfer.rxObj(button, sizeof(button), sizeof(messageType));
            switch (button) {
                case 'u':
                    state = AUTO;
                    navigation.currentWaypoint = 0;
                    navigation.finished = false;
                    screen.setMode(Screen::Mode::WAYPOINTS);
                    screen.showScreen();
                    break;
                case 'd':
                    state = RC;
                    screen.setMode(Screen::Mode::RUNNING);
                    screen.showScreen();
                    break;
                case 'l':
                    //zero heading
                    navigation.poseOffset.heading = robotStatus.pose.heading;
                    break;
                case 'r':
                    //zero position
                    navigation.poseOffset.x = robotStatus.pose.x;
                    navigation.poseOffset.y = robotStatus.pose.y;
                    break;
            }
            break;
        default:
           break;
    }
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
    screen.setMode(Screen::Mode::DEBUG_WARNING);
    screen.showScreen();
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
    int32_t buttonThreshold = 30;  //1024 should be supply voltage, button pulls pin low
    bool shouldRunPost = false;
    if (analogRead(TEENSY_PIN_BUTTON) < buttonThreshold) {
        shouldRunPost = true;
        screen.setMode(Screen::Mode::POST_START);
        screen.showScreen();
        delay(500);
    }
    // do i2c scan
    hal.doI2CScan();
    if (shouldRunPost) {
        screen.setMode(Screen::Mode::POST_I2C);
        screen.showScreen();
        delay(4000);
    }

    // Initialise motors
    hal.initialiseMotors();
    if (shouldRunPost) {
        screen.setMode(Screen::Mode::POST_MOTORS);
        screen.showScreen();
        delay(500);
    }

    // Initialise serial transfer
    myTransfer.begin(Serial2);
    if (shouldRunPost) {
        myTransfer.begin(Serial2);
        screen.showScreen();
        delay(500);
    }

    // Initialise ToF sensors
    hal.initialiseTOFSensors();
    if (shouldRunPost) {
        screen.setMode(Screen::Mode::POST_TOF);
        screen.showScreen();
        delay(4000);
    }

    // //initialise IMU
    // TODO - should we halt here if the IMU fails?
    hal.initialiseIMU();

    screen.setMode(Screen::Mode::POST_IMU_INIT_STATUS);
    screen.showScreen();
    delay(500);

    /*
    If it is initialised:
    * attempt to load the saved config from EEPROM, report results
    * perform a calibration movement, until the system reports that it is calibrated
    * saved the calibration data to the EEPROM
    */
    if (robotStatus.activation.imu) {
        // attempt to restore calibration data
        screen.setMode(
            hal.restoreCalibrationData() ? Screen::Mode::POST_IMU_CALIBRATION_RESTORE_OK : Screen::Mode::POST_IMU_CALIBRATION_RESTORE_FAIL);
        screen.showScreen();
        delay(1000);

        // perform calibration - repeat until system calibration is true
        while (!hal.calibrateIMU()) {
            // update the imuEvent in the status object
            hal.getIMUEvent();
            // delay by the defined sample rate
            delay(BNO055_SAMPLERATE_DELAY_MS);
            screen.setMode(Screen::Mode::POST_IMU_CALIBRATE_WITH_EVENT);
            screen.showScreen();
        }

        // save the config to EEPROM
        hal.saveCalibrationData();
        screen.setMode(Screen::Mode::POST_IMU_CALIBRATION_SAVED);
        screen.showScreen();
        delay(2000);

        //  show calibration complete screen
        screen.setMode(Screen::Mode::POST_IMU_CALIBRATION_COMPLETE);
        screen.showScreen();
        delay(500);
    }
}

void loop() {

    if ((state == AUTO) && (challengeTimeout.hasPassed(UPDATE_CHALLENGE_TIMEOUT_MS))){
        Speeds requestedMotorSpeeds;
        requestedMotorSpeeds = navigation.update(robotStatus.pose, robotStatus.previousOrientation.x, 0.02);
        hal.setRequestedMotorSpeeds(requestedMotorSpeeds);
        robotStatus.waypointPose = navigation.currentPose;
        robotStatus.wayDist = navigation.distanceToGo;
        robotStatus.wayHead = navigation.headingError;
        robotStatus.numWaypoints = navigation.numOfWaypoints;
        challengeTimeout.restart();
    }

    // Do we need to update the various sensors?
    if (updateEncodersTimeout.hasPassed(UPDATE_ENCODER_TIMEOUT_MS)) {
        // Read Encoder counts
        hal.updateEncoders();
        // restart timer
        updateEncodersTimeout.restart();
    }

    if (updateIMUTimeout.hasPassed(UPDATE_IMU_TIMEOUT_MS)) {
        hal.updateIMU();
        // restart timer
        updateIMUTimeout.restart();
    }

    if (updateTOFTimeout.hasPassed(UPDATE_TOF_TIMEOUT_MS)) {
        hal.updateTOFSensors();
        // restart timer
        updateTOFTimeout.restart();
    }

    // Is there an incoming message available?
    if (myTransfer.available()) {
        processMessage(myTransfer);
    }

    // if the message sending timeout has passed then increment the missed count
    // and reset
    if (receiveMessageTimeout.hasPassed(MOTOR_MESSAGE_TIMEOUT_MS)) {
        robotStatus.incrementMissedMotorCount();
        receiveMessageTimeout.restart();
    }

    if (robotStatus.batteryIsLow() || robotStatus.motorMessageCommsDown()) {
        screen.setMode(Screen::Mode::ERROR);

        // TODO move this outside the if block when we're only using the screen class for displaying info
        screen.showScreen();

        // Set motors to dead stop
        hal.stopMotors();
    } else {
        if (state == AUTO){
            screen.setMode(Screen::Mode::WAYPOINTS);
        } else {
            screen.setMode(Screen::Mode::RUNNING);
        }
        screen.showScreen();
    }

    if (sendSensorDataTimeout.hasPassed(SEND_SENSOR_DATA_TIMEOUT_MS)) {
        sendSensorDataTimeout.restart();
        uint16_t payloadSize = 0;

        //update odometry
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

    // Update motor speeds
    hal.updateMotorSpeeds();
}