#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>
#include <Arduino.h>
#include <unordered_map>
#include <Chrono.h>
#include <EEPROM.h>
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

// #define DEBUG

Servo motorLeft;
Servo motorRight;
Servo esc_1;
Servo esc_2;

struct Pose {
    float heading;
    float x;
    float y;
} currentPosition, previousPosition;

struct Speeds {
    float left;
    float right;
};
float averageSpeed;
float minSpeed = 0.0001;

Speeds deadStop = {0, 0};

long lastLoopTime = millis();
float loopTime = 0;
uint32_t missedMotorMessageCount = 0;

float minBatVoltage = 11.1;
float trackWidth = 136;

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

Adafruit_ADS1115 ads1115(ADC_ADDR);	// minesweeper ADC

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

struct Pose updatePose(struct Pose oldPosition, float heading, float distanceTravelled){
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

float batteryVoltage(){
    //reads ADC, interprets it and
    //returns battery voltage as a float
    float ADC, voltage;
    //AnalogRead returns 10bit fraction of Vdd
    ADC = analogRead(TEENSY_PIN_BATT_SENSE)*3.3/1023.0;

     //ADC reads battery via a potential divider of 33k and 10k
     //but they're wrong/outof spec
    voltage = ADC * (26.9+10.0)/10.0+4;
    return voltage;
}

float getDistanceTravelled(){
   // Uses minimum encoder reading to estimate actual travel speed. returns a speed struct
    float travelPerEncoderCount = 1.0;           //millimeters per encoder count. from testing
    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec
    float wheelTravel[NUM_ENCODERS];
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        wheelTravel[n] = ((float)(encoderReadings[n] - oldEncoderReadings[n])) * travelPerEncoderCount;
    }
    float rightTravel = minMagnitude(wheelTravel[3], wheelTravel[5], wheelTravel[5]);
    float leftTravel = minMagnitude(wheelTravel[0], wheelTravel[1], wheelTravel[1]);
    return (leftTravel-rightTravel)/2;
}

struct Speeds feedForward(struct Speeds targetSpeeds){
    // takes two speed commands in mm/sec
    // returns predicted motor power -100 to +100%
    //inputs and outputs both Speed structs

    struct Speeds commandSpeeds;

    float minTurnPower = 4;  //determined from practical testing
    float minForwardPower = 5;  //same
    float powerCoefficient = 45;  //same
    float turnThreshold = 500;  //units: mm/sec. arbitary, value.
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
            commandSpeeds.right = sgn(targetSpeeds.right) * abs(targetSpeeds.right) / powerCoefficient + minForwardPower;
            commandSpeeds.left = sgn(targetSpeeds.left) * abs(targetSpeeds.left) / powerCoefficient + minForwardPower;
        }
    } else {
        //if we're not trying to move, turn the motors off
        commandSpeeds.right = 0;
        commandSpeeds.left = 0;
    }
    return commandSpeeds;
}

struct Speeds PID(struct Speeds targetSpeeds, struct Speeds commandSpeeds){
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
    float travelPerEncoderCount = 0.4;           //millimeters per encoder count. from testing

    //work out target turn rate
    float targetTurnRate = (targetSpeeds.left - targetSpeeds.right) / trackWidth;

    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec
    float motorSpeeds[NUM_ENCODERS];
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        motorSpeeds[n] = ((float)(encoderReadings[n] - oldEncoderReadings[n])) / loopTime * travelPerEncoderCount;
    }

    //most representative speed assumed to be slowest wheel
    //#0, #1 & #3 is left,  #3, #4 & #5 is right
    //#0 is front left     #3 is front right
    //#1 is rear left      #4 is middle right
    //#2 is middle left    #5 is rear right
    Speeds actualMotorSpeeds;

    actualMotorSpeeds.right = minMagnitude(motorSpeeds[3], motorSpeeds[4], motorSpeeds[5]);
    actualMotorSpeeds.left = minMagnitude(motorSpeeds[0], motorSpeeds[1], motorSpeeds[2]);

    //work out actual turn rate
    float actualTurnRate = wrapTwoPi(orientationReading.x - oldOrientationReading.x) / loopTime;

    display.println(" ");
    display.printf("P in L:%3.0f,  A R:%3.0f", commandSpeeds.left, commandSpeeds.right);
    // do actual Proportional calc.
    //speed error is target - actual.
    float fwdKp = 0.02;  //ie. how much power to use for a given speed error
    //apply P correction. right encoder reads negative when going forwards.
    // right motor power inverted when eventually sent, so here we just need to apply more (+) power if slow
    commandSpeeds.left += fwdKp * (targetSpeeds.left - actualMotorSpeeds.left);
    commandSpeeds.right += fwdKp * (targetSpeeds.right + actualMotorSpeeds.right);
    float turnKp = 0;
    float steeringCorrection = turnKp * (targetTurnRate - actualTurnRate);
    display.println(" ");
    display.printf("t L:%3.0f,   t R:%3.0f", targetSpeeds.left, targetSpeeds.right);
    display.println(" ");
    display.printf("t L:%3.0f,   a R:%3.0f", actualMotorSpeeds.left, actualMotorSpeeds.right);
    display.println(" ");
    display.printf("P outL:%3.0f, A R:%3.0f", commandSpeeds.left, commandSpeeds.right);
    //display.println(" ");
    //display.printf("target rate:%2.2f", targetTurnRate);
    //display.println(" ");
    //display.printf("actual rate:%2.2f", actualTurnRate);
    //display.println(" ");
    //display.printf("steering correction: %2.2f", steeringCorrection);
    //display.println(" ");
    //display.printf("heading: %2.2f", orientationReading.x);
    display.display();
    commandSpeeds.left += steeringCorrection;
    commandSpeeds.right -= steeringCorrection;

    //constrain output
    float max_power=65;
    commandSpeeds.left =max(min(commandSpeeds.left, max_power), -max_power);
    commandSpeeds.right =max(min(commandSpeeds.right, max_power), -max_power);

    return commandSpeeds;
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
            //C++20 has a contains() method for unordered_map
            // but find() is only one available to us?
            if (I2C_ADDRESS_NAMES.find(addr) != I2C_ADDRESS_NAMES.end()){
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
    float maxspeed_mm_per_sec = 1500;  //max acheivable is 8000
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
    averageSpeed = 0.7 * averageSpeed + 0.3 * (abs(commandMotorSpeeds.left) + abs(commandMotorSpeeds.right)); 

    //if its been zero for a while, just stop, else work out the PID modified speeds
    //not applying PID when stopped, stops the motors going crazy if the robot is carried
    if (averageSpeed < minSpeed) {
      commandMotorSpeeds = deadStop;
    } else {
      //apply PID to motor powers based on deviation from target speed
      commandMotorSpeeds = PID(targetMotorSpeeds, commandMotorSpeeds);
    }
    
    //display.println(" ");
    //display.printf("requested L:%3.0f", requestedMotorSpeeds.left);
    //display.println(" ");
    //display.printf("requested R:%3.0f", requestedMotorSpeeds.right);
    //display.println(" ");
    //display.printf("target L:%3.0f", targetMotorSpeeds.left);
    //display.println(" ");
    //display.printf("target R:%3.0f", targetMotorSpeeds.right);
    //display.println(" ");
    //display.printf("command L:%2.2f", commandMotorSpeeds.left);
    //display.println(" ");
    //display.printf("command R:%2.2f", commandMotorSpeeds.right);
    //display.display();
    motorLeft.writeMicroseconds(map(commandMotorSpeeds.left, -100, 100, 1000, 2000));
    motorRight.writeMicroseconds(map(commandMotorSpeeds.right * -1, -100, 100, 1000, 2000));
}

void processMessage(SerialTransfer &transfer) {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;

    // Get message type, indicated by the first byte of the message
    uint8_t messageType ;
    recSize = myTransfer.rxObj(messageType, recSize);
    switch (messageType) {
        // 0 - motor speed message
        case 1:
            Speeds requestedMotorSpeeds;
//            float messages[4];
//            transfer.rxObj(messages, sizeof(messages), sizeof(messageType));
//            requestedMotorSpeeds = {messages[0], messages[1]};
            transfer.rxObj(requestedMotorSpeeds, recSize);
            setMotorSpeeds(requestedMotorSpeeds, motorLeft, motorRight);
            // reset the missed motor mdessage count
            resetMissedMotorCount();
            // We received a valid motor command, so reset the timer
            receiveMessage.restart();
            display.printf("requested motor speeds: %2.2f, %2.2f",requestedMotorSpeeds.left,requestedMotorSpeeds.right);
            display.display();
            break;
        case 2:
            char button;
            transfer.rxObj(button, sizeof(button), sizeof(messageType));
            switch (button) {
                case 'c':
                    esc_1.writeMicroseconds(900);
                    display.println(F("jaw closing"));
                    display.display();
                    break;
                case 'x':
                    esc_2.writeMicroseconds(1300);
                    display.println(F("jaw down"));
                    display.display();
                    break;
                case 's':
                    esc_1.writeMicroseconds(1600);
                    display.println(F("jaw opening"));
                    display.display();
                    break;
                case 't':
                    esc_2.writeMicroseconds(2100);
                    display.println(F("jaw up"));
                    display.display();
                    break;
            }
            break;
        default:
            display.printf("invalid message type received %i", messageType);
            display.display();
            delay(500);        
    }
}

void post(){
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

    //initialise ADC
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("ADC"));
    ads1115.begin();
    display.println(F("Initialised"));
    display.println(F("First reading:"));
    for (u_int8_t n = 0; n < 4; n++) {
        lightSensors[n] = ads1115.readADC_SingleEnded(n);
    }
    display.printf("FL: %d      FR: %d", lightSensors[2], lightSensors[0]);
    display.println();
    display.printf("BL: %d      BR: %d", lightSensors[1], lightSensors[3]);
    display.display();
    delay(2000);

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
        display.setCursor(0,0);
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
        display.setCursor(0,0);

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
    display.setCursor(0,0);
    display.println("Running");
    display.display();
    currentPosition.heading = currentPosition.x = currentPosition.y = 0;
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
    display.printf("%2.2f V", batteryVoltage());
    display.display();
    delay(2000);
    int buttonThreshold = 30;  //1024 should be supply voltage, button pulls pin low
    bool enterPost = false;
    if (analogRead(TEENSY_PIN_BUTTON) < buttonThreshold){
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
        //initialise ADC
        ads1115.begin();

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
}

void loop() {

    // Is there an incoming message available?
    if (myTransfer.available()) {
        processMessage(myTransfer);
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

        for (u_int8_t n = 0; n < 4; n++) {
            lightSensors[n] = 0; //ads1115.readADC_SingleEnded(n);
        }
        uint16_t payloadSize = 0;

        //update odometry
        previousPosition = currentPosition;
        float distanceMoved = getDistanceTravelled();
        currentPosition = updatePose(previousPosition, orientationReading.x, distanceMoved);

        // Prepare the distance data
        payloadSize = myTransfer.txObj(distances, payloadSize);

        //Prepare encoder data
        payloadSize = myTransfer.txObj(encoderReadings, payloadSize);

        //Prepare IMU data
        payloadSize = myTransfer.txObj(orientationReading, payloadSize);

        //Prepare odometry data
        payloadSize = myTransfer.txObj(currentPosition, payloadSize);

        //Prepare ADC data
        payloadSize = myTransfer.txObj(lightSensors, payloadSize);

        // Send data
        myTransfer.sendData(payloadSize);
    }
}