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
#include <math.h>

#include <unordered_map>

#include "Wire.h"
#include "config.h"
#include "graphics.h"

// #define DEBUG

#ifndef ARDUINO_TEENSY31
HardwareSerial Serial2(USART2);
#endif

Servo motorLeft;
Servo motorRight;
Servo esc_1;
Servo esc_2;

struct Pose {
    float heading;
    float x;
    float y;
} currentPosition, previousPosition;
float headingOffset=0;
struct Speeds {
    float left;
    float right;
};
float averageSpeed;
float minSpeed = 20;
struct Pose waypoints[3];
uint8_t currentWaypoint=0;
bool navigating = false;

Speeds deadStop = {0, 0};

long lastLoopTime = millis();
float loopTime = 0;
uint32_t missedMotorMessageCount = 0;

float minBatVoltage = 11.1;
float trackWidth = 136;
float travelPerEncoderCount = 0.262;  //millimeters per encoder count. from testing

SerialTransfer myTransfer;

int8_t step = 1;

Chrono receiveMessage;
Chrono readSensors;
VL53L0X sensor;

float distances[8];
bool activeToFSensors[8];
int16_t lightSensors[4];

Encoder encoders[NUM_ENCODERS] = {
    Encoder(TEENSY_PIN_ENC1A, TEENSY_PIN_ENC1B),
    Encoder(TEENSY_PIN_ENC2A, TEENSY_PIN_ENC2B),
    Encoder(TEENSY_PIN_ENC3A, TEENSY_PIN_ENC3B),
    Encoder(TEENSY_PIN_ENC4A, TEENSY_PIN_ENC4B),
    Encoder(TEENSY_PIN_ENC5A, TEENSY_PIN_ENC5B),
    Encoder(TEENSY_PIN_ENC6A, TEENSY_PIN_ENC6B)};

long encoderReadings[NUM_ENCODERS];
long oldEncoderReadings[NUM_ENCODERS];

Adafruit_SSD1306 display(128, 64);

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_ADDR);
struct OrientationReading {
    float x;
    float y;
    float z;
} orientationReading, oldOrientationReading;

void tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }

    Wire.beginTransmission(TCA_ADDR);
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

struct Pose updatePose(struct Pose oldPosition, float heading, float distanceTravelled) {
    // takes current position, new heading and distance traveled to work out a new position
    struct Pose newPosition;
    newPosition.heading = heading;
    newPosition.x = oldPosition.x + distanceTravelled * cos(heading);
    newPosition.y = oldPosition.y + distanceTravelled * sin(heading);
    return newPosition;
}

float wrapTwoPi(float angle) {
    //wraps an angle to stay within +/-pi
    while (angle > M_PI) angle -= TWO_PI;
    while (angle < -M_PI) angle += TWO_PI;
    return angle;
}

float batteryVoltage() {
    //reads ADC, interprets it and
    //returns battery voltage as a float
    float adcReading, voltage;
    //AnalogRead returns 10bit fraction of Vdd
    adcReading = analogRead(TEENSY_PIN_BATT_SENSE) * 3.3 / 1023.0;

     //ADC reads battery via a potential divider of 33k and 10k
     //but they're wrong/out of spec ((33+10)/10 = 4.3)
    voltage = adcReading * 3.71;
    return voltage;
}
Speeds getWheelTravel() {
    // Uses minimum encoder reading to estimate actual travel speed.
    // returns a speed struct of wheel travel in mm

    //compare old and latest encoder readings to see how much each wheel has rotated
    float wheelTravel[NUM_ENCODERS];
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        wheelTravel[n] = ((float)(encoderReadings[n] - oldEncoderReadings[n])) * travelPerEncoderCount;
    }
    //most representative speed assumed to be slowest wheel
    //#0, #1 & #3 is left,  #3, #4 & #5 is right
    //#0 is front left     #3 is front right
    //#1 is rear left      #4 is middle right
    //#2 is middle left    #5 is rear right
    Speeds travel;
    travel.right = minMagnitude(wheelTravel[3], wheelTravel[4], wheelTravel[5]);
    travel.left = minMagnitude(wheelTravel[0], wheelTravel[1], wheelTravel[2]);
    return travel;
}
float getDistanceTravelled() {
    //returns the average distance travelled by right and left wheels, in mm
    Speeds travel = getWheelTravel();
    return (travel.left - travel.right) / 2;
}

struct Speeds feedForward(struct Speeds targetSpeeds) {
    // takes two speed commands in mm/sec
    // returns predicted motor power -100 to +100%
    //inputs and outputs both Speed structs

    struct Speeds commandSpeeds;

    float minTurnPower = 4;       //determined from practical testing
    float minForwardPower = 5;     //same
    float powerCoefficient = 50;  //same
    float turnThreshold = 100;     //units: mm/sec. arbitary, value.
    // using the turnThreshold does create a discontinuity when transitioning
    // from mostly straight ahead to a slight turn but then the two moves
    // do need different power outputs. maybe linear interpolation between
    // the two would be better?

    // since there's a min power needed to move (as defined above)
    // first check if we're trying to move
    if (targetSpeeds.left != 0 and targetSpeeds.right != 0) {
        //then check if we're trying to turn or not, i.e. left and right speeds different
        if (abs(targetSpeeds.right - targetSpeeds.left) > turnThreshold) {
            //then predict power needed to acheive that speed. formule derived from curve fitting experimental results
            float turnComponent = sgn(targetSpeeds.right - targetSpeeds.left) * (abs(targetSpeeds.right - targetSpeeds.left) / powerCoefficient + minTurnPower);
            float forwardComponent = (targetSpeeds.right + targetSpeeds.left) / 2 / powerCoefficient;
            commandSpeeds.right = turnComponent + forwardComponent;
            commandSpeeds.left = -turnComponent + forwardComponent;
        } else {
            //a different formula is best fit for going straight
            commandSpeeds.right = sgn(targetSpeeds.right) * (abs(targetSpeeds.right) / powerCoefficient + minForwardPower);
            commandSpeeds.left = sgn(targetSpeeds.left) * (abs(targetSpeeds.left) / powerCoefficient + minForwardPower);
        }
    } else {
        //if we're not trying to move, turn the motors off
        commandSpeeds.right = 0;
        commandSpeeds.left = 0;
    }
    return commandSpeeds;
}

struct Speeds PID(struct Speeds targetSpeeds, struct Speeds commandSpeeds) {
    // apply PID
    // takes two speed commands in -100 to +100 and two
    // target speeds in mm/sec
    // uses sensor feedback to correct for errors
    // returns motor power -100 to +100%
    //inputs and outputs all Speed structs

    // or at the moment, just proportional
    //. i.e power percentage proporational to difference
    // between desired speed and current actual wheel speed

    float loopTime = (millis() - lastLoopTime) / 1000.0;  // divide by 1000 converts to seconds.
    lastLoopTime = millis();

    //work out target turn rate
    float targetTurnRate = (targetSpeeds.left - targetSpeeds.right) / trackWidth;

    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec
    Speeds travel = getWheelTravel();
    Speeds actualMotorSpeeds;
    actualMotorSpeeds.right = travel.right/loopTime;
    actualMotorSpeeds.left = travel.left/loopTime;

    //work out actual turn rate
    float actualTurnRate = wrapTwoPi(orientationReading.x - oldOrientationReading.x) / loopTime;

    //display.println(" ");
    //display.printf("P in L:%3.0f,  A R:%3.0f", commandSpeeds.left, commandSpeeds.right);
    // do actual Proportional calc.
    //speed error is target - actual.
    float fwdKp = 0.01;  //ie. how much power to use for a given speed error
    //apply P correction. right encoder reads negative when going forwards.
    // right motor power inverted when eventually sent, so here we just need to apply more (+) power if slow
    commandSpeeds.left += fwdKp * (targetSpeeds.left - actualMotorSpeeds.left);
    commandSpeeds.right += fwdKp * (targetSpeeds.right + actualMotorSpeeds.right);
    float turnKp = 2;
    float steeringCorrection = turnKp * (targetTurnRate - actualTurnRate);
    commandSpeeds.left += steeringCorrection;
    commandSpeeds.right -= steeringCorrection;

    //constrain output
    float max_power = 65;
    commandSpeeds.left = max(min(commandSpeeds.left, max_power), -max_power);
    commandSpeeds.right = max(min(commandSpeeds.right, max_power), -max_power);

    return commandSpeeds;
}

void
haltAndCatchFire() {
    while (1) {
    }
}

void do_i2c_scan() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("I2c Devices");
    for (uint8_t addr = 1; addr <= 127; addr++) {
        Wire.beginTransmission(addr);
        if (!Wire.endTransmission() == 0) {
            //C++20 has a contains() method for unordered_map
            // but find() is only one available to us?
            if (I2C_ADDRESS_NAMES.find(addr) != I2C_ADDRESS_NAMES.end()) {
                display.println(I2C_ADDRESS_NAMES.at(addr));
            } else {
                display.print("0x");
                display.println(addr, HEX);
            }
        }
    }
    display.display();
    delay(4000);
}

void incrementMissedMotorCount() {
    missedMotorMessageCount++;
}

void resetMissedMotorCount() {
    missedMotorMessageCount = 0;
}

void setMotorSpeeds(Speeds requestedMotorSpeeds, Servo &motorLeft, Servo &motorRight) {
    Speeds commandMotorSpeeds, targetMotorSpeeds;

    //convert -100 - +100 percentage speed command into mm/sec
    // for autonomous control we could revert back to using full scale
    // but for manual control, and for testing speedcontrol precision
    // better to start with limiting to lower speeds
    float maxspeed_mm_per_sec = 1000;  //max acheivable is ~3200
    targetMotorSpeeds.right = requestedMotorSpeeds.right * maxspeed_mm_per_sec / 100;
    targetMotorSpeeds.left = requestedMotorSpeeds.left * maxspeed_mm_per_sec / 100;

    //convert speed commands into predicted power
    // otherwise known as feedforward. We can do feedforward
    // and/or PID speed control. both is better but either
    // alone gives functional results

    //get predicted motor powers from feedforward
    commandMotorSpeeds = feedForward(targetMotorSpeeds);

    // check if the command speed has been close to zero for a while
    // if it is, we're probably are stopped and want to be stopped
    averageSpeed = 0.5 * averageSpeed + 0.5 * (abs(commandMotorSpeeds.left) + abs(commandMotorSpeeds.right)); 

    //if its been zero for a while, just stop, else work out the PID modified speeds
    //not applying PID when stopped, stops the motors going crazy if the robot is carried
    if (averageSpeed < minSpeed) {
        commandMotorSpeeds = deadStop;
    } else {
        //apply PID to motor powers based on deviation from target speed
        commandMotorSpeeds = PID(targetMotorSpeeds, commandMotorSpeeds);
    }
    motorLeft.writeMicroseconds(map(commandMotorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(commandMotorSpeeds.right * -1, -100, 100, 1000, 2000));
}

float distanceToWaypoint(Pose target, Pose current){
    //returns distance 'as the crow flies' to the target pose 
    float distance;
    //hypotenuse of dx, dy triangle gives distance, using h^2=x^2+y^2
    distance = sqrt(powf((target.x-current.x),2) + powf((target.y-current.y),2));
    display.printf("distance: %2.2f", distance);
    return distance;
}

float headingToWaypoint(Pose target, Pose current){
    float dx, dy, relativeHeading;
    dx = target.x-current.x;
    dy = target.y-current.y;
    if (dy != 0) {
        relativeHeading = (float) atan2(dy, dx);
    } else {
        relativeHeading = sgn(dy) * M_PI/2;
    }
    relativeHeading = wrapTwoPi(relativeHeading - current.heading);

    return relativeHeading;
}

void navigate(){
    Speeds MotorSpeeds;
    if (distanceToWaypoint(waypoints[2], currentPosition) < 300) {   
        MotorSpeeds.left = MotorSpeeds.right = 0;
        navigating = false;
    } else {
        MotorSpeeds.left = MotorSpeeds.right = 30;
    }
    setMotorSpeeds(MotorSpeeds, motorLeft, motorRight);
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
            Speeds requestedMotorSpeeds;
//            float messages[4];
//            transfer.rxObj(messages, sizeof(messages), sizeof(messageType));
//            requestedMotorSpeeds = {messages[0], messages[1]};
            transfer.rxObj(requestedMotorSpeeds, recSize);
            if (!navigating){
                setMotorSpeeds(requestedMotorSpeeds, motorLeft, motorRight);
            }
            // reset the missed motor mdessage count
            resetMissedMotorCount();
            // We received a valid motor command, so reset the timer
            receiveMessage.restart();
            break;
        case 2:
            char button;
            transfer.rxObj(button, sizeof(button), sizeof(messageType));
            switch (button) {
                case 'c':
                    esc_1.writeMicroseconds(900);
                    display.println(F("jaw closing"));
                    display.display();
                    delay(200);
                    break;
                case 'x':
                    esc_2.writeMicroseconds(1300);
                    display.println(F("jaw down"));
                    display.display();
                    delay(200);
                    break;
                case 's':
                    esc_1.writeMicroseconds(1600);
                    display.println(F("jaw opening"));
                    display.display();
                    delay(200);
                    break;
                case 't':
                    esc_2.writeMicroseconds(2100);
                    display.println(F("jaw up"));
                    display.display();
                    delay(200);
                    break;
                case 'l':
                    display.println(F("zeroing heading"));
                    display.display();
                    delay(500);
                    headingOffset = orientationReading.x;
                    currentPosition.heading=0;
                    break;
                case 'r':
                    display.println(F("zeroing odometry"));
                    display.display();
                    delay(500);
                    currentPosition.x=0;
                    currentPosition.y=0;
                    break;
                case 'u':
                    display.println(F("navigating to next waypoint"));
                    display.display();
                    currentWaypoint += 1;
                    currentWaypoint = currentWaypoint % sizeof(waypoints);
                    navigating=true;
                    break;
                case 'd':
                    display.println(F("stopping navigation"));
                    display.display();
                    navigating = false;
                    break;
            }
            break;
        default:
            display.printf("invalid message type received %i", messageType);
            display.display();
            delay(500);        
    }
}

void post() {
    // do i2c scan
    do_i2c_scan();

    // Attach motors
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Motors"));
    display.display();
    motorLeft.attach(TEENSY_PIN_DRIVE_LEFT);
    motorRight.attach(TEENSY_PIN_DRIVE_RIGHT);
    display.setCursor(0, 10);
    display.print("OK");
    display.display();
    delay(500);

    // Initialise serial transfer
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Serial transfer"));

    display.display();
    display.print("OK");
    myTransfer.begin(Serial2);
    display.display();
    delay(500);

    // Initialise ToF sensors
    tcaselect(0);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("ToF sensors"));
    display.display();
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        display.printf("initialising %d", t);
        display.display();
        activeToFSensors[t] = sensor.init();
        display.setCursor(0, display.getCursorY() + 1);
        display.printf("init %d done", t);

        if (activeToFSensors[t]) {
            display.printf("%d: OK", t);
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
            display.printf("%d: FAIL", t);
        }
        if (t % 2 == 1) {
            display.println("");
        } else {
            display.setCursor(64, display.getCursorY());
        }
        display.display();
        delay(50);
    }
    delay(500);
    display.clearDisplay();
    display.display();

    // //initialise IMU
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("IMU"));
    // Do we have an IMU
    if (!bno.begin()) {
        display.println("FAIL");
        display.display();
        delay(3000);
    } else {
        display.println("OK");
        display.display();
        delay(3000);

        display.clearDisplay();
        display.setCursor(0, 0);

        // look for calibration data. if it exists, load it.
        // if not, calibrate then store the data in the EEPROM
        int eeAddress = 0;
        long bnoID;
        EEPROM.get(eeAddress, bnoID);

        adafruit_bno055_offsets_t calibrationData;
        sensor_t sensor;

        bno.getSensor(&sensor);
        if (bnoID != sensor.sensor_id) {
            display.println("No Calibration Data in EEPROM");

        } else {
            display.println("Found Calibration in EEPROM.");
            eeAddress += sizeof(long);
            EEPROM.get(eeAddress, calibrationData);

            display.println("Restoring...");
            bno.setSensorOffsets(calibrationData);

            display.println("Restored");
        }
        display.display();
        delay(1000);

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Move robot now to check calibration");
        display.display();
        delay(5000);
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        sensors_event_t event;
        bno.getEvent(&event);
        display.println("Move sensor to calibrate magnetometers");
        display.display();
        u_int8_t curYPos = display.getCursorY();
        while (!system) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            /* Display the individual values */
            display.setCursor(0, curYPos);
            display.print("Sys:");
            display.print(system, DEC);
            display.print(" G:");
            display.print(gyro, DEC);
            display.print(" A:");
            display.print(accel, DEC);
            display.print(" M:");
            display.println(mag, DEC);
            display.setCursor(0, curYPos + 10);
            /* Display the individual values */
            delay(BNO055_SAMPLERATE_DELAY_MS);
            display.print("X:");
            display.print(event.orientation.x, 4);
            display.print(" Y:");
            display.print(event.orientation.y, 4);
            display.print(" Z:");
            display.println(event.orientation.z, 4);
            display.display();
            bno.getEvent(&event);
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
        display.println("calibrated OK");
        display.display();
        delay(3000);

        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);
        display.clearDisplay();
        display.setCursor(0, 0);

        display.println("Storing calibration data to EEPROM...");

        eeAddress = 0;
        bno.getSensor(&sensor);
        bnoID = sensor.sensor_id;

        EEPROM.put(eeAddress, bnoID);

        eeAddress += sizeof(long);
        EEPROM.put(eeAddress, newCalib);
        display.println("Data stored to EEPROM.");
        display.display();

        delay(2000);
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Running");
    display.display();
    currentPosition.heading = currentPosition.x = currentPosition.y = 0;
}

void setup() {
// Initialise I2C bus
#ifdef ARDUINO_TEENSY31
    Wire.begin();
#else
    Wire.begin(TEENSY_PIN_I2C_SDA, TEENSY_PIN_I2C_SCL);
#endif

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

    pinMode(TEENSY_PIN_BUTTON, INPUT_PULLUP);
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Git Branch:");
    display.println(GIT_BRANCH);
    display.println("Git commit hash:");
    display.println(GIT_REV);
    display.display();
    delay(2000);
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Press button now to  enter POST");
    display.println();
    display.println("Battery Voltage:");
    float batVoltage = batteryVoltage();
    display.printf("%2.2f V", batVoltage);
    display.display();
    delay(2000);
    int buttonThreshold = 30;  //1024 should be supply voltage, button pulls pin low
    bool enterPost = false;
    if (analogRead(TEENSY_PIN_BUTTON) < buttonThreshold) {
        enterPost = true;
        display.println("Entering POST...");
        display.display();
        delay(500);
    }

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

    Serial2.begin(1000000);
    while (!Serial2) {
    };

    if (enterPost) {
        post();
    } else {
        // Attach motors
        motorLeft.attach(TEENSY_PIN_DRIVE_LEFT);
        motorRight.attach(TEENSY_PIN_DRIVE_RIGHT);
        esc_1.attach(TEENSY_PIN_LH_BALL_ESC);
        esc_2.attach(TEENSY_PIN_RH_BALL_ESC);

        // Set motors to stop
        setMotorSpeeds(deadStop, motorLeft, motorRight);

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
        display.clearDisplay();
        display.setCursor(0, 0);
        if (!bno.begin()) {
            display.println("IMU FAIL");
        } else {
            display.println("IMU CAL:");
        }
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;

        u_int8_t curYPos = display.getCursorY();
        while (!system) {
            bno.getCalibration(&system, &gyro, &accel, &mag);
            display.setCursor(0, curYPos);
            /* Display the individual values */
            display.print("Sys:");
            display.print(system, DEC);
            display.print(" G:");
            display.print(gyro, DEC);
            display.print(" A:");
            display.print(accel, DEC);
            display.print(" M:");
            display.println(mag, DEC);
            display.display();
        };
        display.println("calibrated OK");
        display.display();
        delay(200);
    }
    waypoints[0].x = 0;
    waypoints[0].y = -50;
    waypoints[1].x = 500;
    waypoints[1].y = 0;
    waypoints[2].x = 900;
    waypoints[2].y = 0;
}

void loop() {
    // Is there an incoming message available?
    if (myTransfer.available()) {
        processMessage(myTransfer);
    }

    if (navigating) {
        navigate();
    }
    // if the message sending timeout has passed then increment the missed count
    // and reset
    if (receiveMessage.hasPassed(20)) {
        incrementMissedMotorCount();
        receiveMessage.restart();
    }

    bool shouldInvertDisplay = false;
    // Have we missed 10 valid motor messages?

    display.clearDisplay();
    display.setCursor(0, 0);

    if (missedMotorMessageCount >= 10) {
        shouldInvertDisplay = true;
        display.printf("missed message %d", missedMotorMessageCount);
        display.display();
    }
    // is battery going flat?
    if (batteryVoltage() < minBatVoltage) {
        shouldInvertDisplay = true;
        display.printf("low battery");
        display.display();
    }

    display.invertDisplay(shouldInvertDisplay);

    // If we have missed 10 valid motor messages
    // or the battery is going flat
    // set motors to dead stop
    if ((missedMotorMessageCount >= 10) || (batteryVoltage() < minBatVoltage)) {
        setMotorSpeeds(deadStop, motorLeft, motorRight);
    }


    if (readSensors.hasPassed(100)) {
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
            oldEncoderReadings[n] = encoderReadings[n];
            encoderReadings[n] = encoders[n].read();
        }

        // Read IMU
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        oldOrientationReading = orientationReading;
        orientationReading.x = radians(orientationData.orientation.x);
        orientationReading.y = radians(orientationData.orientation.y);
        orientationReading.z = radians(orientationData.orientation.z);

        uint16_t payloadSize = 0;

        //update odometry
        previousPosition = currentPosition;
        float distanceMoved = getDistanceTravelled();
        float relativeHeading = orientationReading.x - headingOffset;
        currentPosition = updatePose(previousPosition, relativeHeading, distanceMoved);

        // Prepare the distance data
        payloadSize = myTransfer.txObj(distances, payloadSize);

        //Prepare encoder data
        payloadSize = myTransfer.txObj(encoderReadings, payloadSize);

        //Prepare IMU data
        payloadSize = myTransfer.txObj(orientationReading, payloadSize);

        //Prepare odometry data
        payloadSize = myTransfer.txObj(currentPosition, payloadSize);

        // Send data
        myTransfer.sendData(payloadSize);
    }
    display.printf("heading: %2.2f", currentPosition.heading);
    display.println(" ");
    display.printf("position: %2.0f, %2.0f", currentPosition.x, currentPosition.y);
    display.display();

}