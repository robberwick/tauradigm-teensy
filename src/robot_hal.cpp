#include "robot_hal.h"

#include <EEPROM.h>
#include <Encoder.h>
#include <VL53L0X.h>

#include "config.h"
#include "types.h"
#include "utils.h"

RobotHal::RobotHal(Status &status, TwoWire *twi) : _status(status), wire(twi ? twi : &Wire) {
    bno = Adafruit_BNO055(55, IMU_ADDR, wire);

    initialiseEncoders();
};

bool RobotHal::initialiseMotors() {
    // attach motors and set to dead stop
    motors.left.attach(TEENSY_PIN_DRIVE_LEFT);
    motors.right.attach(TEENSY_PIN_DRIVE_RIGHT);
    stopMotors();
    return true;
};

void RobotHal::stopMotors() {
    setRequestedMotorSpeeds(_deadStop);
}

void RobotHal::setRequestedMotorSpeeds(Speeds requestedMotorSpeeds) {
    _status.speeds.requestedSpeed = requestedMotorSpeeds;
}

void RobotHal::updateMotorSpeeds() {
    Speeds commandMotorSpeeds, targetMotorSpeeds;

    //convert -100 - +100 percentage speed command into mm/sec
    // for autonomous control we could revert back to using full scale
    // but for manual control, and for testing speedcontrol precision
    // better to start with limiting to lower speeds
    float maxspeed_mm_per_sec = 1000;  //max acheivable is ~3200
    targetMotorSpeeds.right = _status.speeds.requestedSpeed.right * maxspeed_mm_per_sec / 100;
    targetMotorSpeeds.left = _status.speeds.requestedSpeed.left * maxspeed_mm_per_sec / 100;

    //convert speed commands into predicted power
    // otherwise known as feedforward. We can do feedforward
    // and/or PID speed control. both is better but either
    // alone gives functional results

    //get predicted motor powers from feedforward
    commandMotorSpeeds = feedForward(targetMotorSpeeds);

    // check if the command speed has been close to zero for a while
    _status.speeds.averageSpeed = 0.5 * _status.speeds.averageSpeed + 0.5 * (abs(commandMotorSpeeds.left) + abs(commandMotorSpeeds.right));

    //if its been zero for a while, just stop, else work out the PID modified speeds
    if (_status.speeds.averageSpeed < _minSpeed) {
        commandMotorSpeeds = _deadStop;
    } else {
        //apply PID to motor powers based on deviation from target speed
        commandMotorSpeeds = PID(targetMotorSpeeds, commandMotorSpeeds);
    }

    // save the current commanded motor speeds to the status obj
    _status.speeds.currentSpeed = commandMotorSpeeds;

    motors.left.writeMicroseconds(map(commandMotorSpeeds.left, -100, 100, 1000, 2000));
    motors.right.writeMicroseconds(map(commandMotorSpeeds.right * -1, -100, 100, 1000, 2000));
}

Speeds RobotHal::feedForward(Speeds targetSpeeds) {
    // takes two speed commands in mm/sec
    // returns predicted motor power -100 to +100%
    //inputs and outputs both Speed structs

    struct Speeds commandSpeeds;

    float minTurnPower = 4;       //determined from practical testing
    float minForwardPower = 5;    //same
    float powerCoefficient = 50;  //same
    float turnThreshold = 100;    //units: mm/sec. arbitary, value.
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

Speeds RobotHal::PID(Speeds targetSpeeds, Speeds commandSpeeds) {
    // apply PID
    // takes two speed commands in -100 to +100 and two
    // target speeds in mm/sec
    // uses sensor feedback to correct for errors
    // returns motor power -100 to +100%
    //inputs and outputs all Speed structs

    // or at the moment, just proportional
    //. i.e power percentage proporational to difference
    // between desired speed and current actual wheel speed
    static long lastLoopTime;
    float loopTime = (millis() - lastLoopTime) / 1000.0;  // divide by 1000 converts to seconds.
    lastLoopTime = millis();

    //work out target turn rate
    float targetTurnRate = (targetSpeeds.left - targetSpeeds.right) / _trackWidth;

    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec
    Speeds travel = getWheelTravel();
    Speeds actualMotorSpeeds;
    actualMotorSpeeds.right = travel.right / loopTime;
    actualMotorSpeeds.left = travel.left / loopTime;

    //work out actual turn rate
    // TODO - do this in the status obj? How to get loop time?
    float actualTurnRate = wrapTwoPi(_status.orientation.x - _status.previousOrientation.x) / loopTime;

    // do actual Proportional calc.
    //speed error is target - actual.
    float fwdKp = 0.01;  //ie. how much power to use for a given speed error
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

Speeds RobotHal::getWheelTravel() {
    // Uses minimum encoder reading to estimate actual travel speed.
    // returns a speed struct of wheel travel in mm

    //compare old and latest encoder readings to see how much each wheel has rotated
    float wheelTravel[NUM_ENCODERS];
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        wheelTravel[n] = ((float)(_status.sensors.encoders.current[n] - _status.sensors.encoders.previous[n])) * _travelPerEncoderCount;
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

float RobotHal::getDistanceTravelled() {
    //returns the average distance travelled by right and left wheels, in mm
    Speeds travel = getWheelTravel();
    return (travel.left - travel.right) / 2;
}

void RobotHal::initialiseTOFSensors() {
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        bool sensorActivated = sensor.init();
        _status.activation.tofSensors[t] = sensorActivated;

        if (sensorActivated) {
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
}

void RobotHal::updateTOFSensors() {
    // Iterate through ToF sensors and attempt to get reading
    for (uint8_t t = 0; t < 8; t++) {
        tcaselect(t);
        if (_status.activation.tofSensors[t]) {
            _status.sensors.tofDistances[t] = sensor.readRangeContinuousMillimeters();
            // TODO we should aim to keep this timeout to a minimum - flag occurrences?
            if (sensor.timeoutOccurred()) {
                _status.sensors.tofDistances[t] = 0;
            }
        } else {
            _status.sensors.tofDistances[t] = 0;
        }
    }
}

void RobotHal::tcaselect(uint8_t i) {
    if (i > 7) {
        return;
    }
    wire->beginTransmission(TCA_ADDR);
    wire->write(1 << i);
    wire->endTransmission();
}

void RobotHal::initialiseIMU() {
    _status.activation.imu = bno.begin();
};

bool RobotHal::calibrateIMU() {
    bno.getCalibration(
        &_status.sensors.imu.calibration.system,
        &_status.sensors.imu.calibration.gyro,
        &_status.sensors.imu.calibration.accel,
        &_status.sensors.imu.calibration.mag);

    return (bool)_status.sensors.imu.calibration.system;
}

bool RobotHal::getIMUEvent() {
    return bno.getEvent(&_status.sensors.imu.imuEvent);
}

bool RobotHal::restoreCalibrationData() {
    int eeAddress = 0;
    long bnoID;
    sensor_t sensor;
    adafruit_bno055_offsets_t calibrationData;

    // Attempt to get the sensor ID from the EEPROM
    EEPROM.get(eeAddress, bnoID);
    // Get sensor ID from sensor
    bno.getSensor(&sensor);

    // If they don't match, we don't have calibration data so bail.
    if (bnoID != sensor.sensor_id) {
        return false;
    }

    // offset read address by the size of a long (the sensor ID)
    eeAddress += sizeof(long);
    // populate calibrationData from EEPROM
    EEPROM.get(eeAddress, calibrationData);
    // Apply offsets to IMU
    bno.setSensorOffsets(calibrationData);
    return true;
}

void RobotHal::saveCalibrationData() {
    int eeAddress = 0;
    long bnoID;
    sensor_t sensor;
    adafruit_bno055_offsets_t newCalib;

    // get calibration offsets from imu
    bno.getSensorOffsets(newCalib);

    // get the sensor ID from imu
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    // sotre sensor ID in EEPROM
    EEPROM.put(eeAddress, bnoID);

    // store calibration offsets in EEPROM
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
}

void RobotHal::doI2CScan() {
    for (uint8_t addr = 1; addr <= 127; addr++) {
        wire->beginTransmission(addr);
        wire->write(0);
        _status.activation.i2c[addr] = (wire->endTransmission() == 0);
    }
}
void RobotHal::initialiseEncoders() {
    encoders[0] = new Encoder(TEENSY_PIN_ENC1A, TEENSY_PIN_ENC1B);
    encoders[1] = new Encoder(TEENSY_PIN_ENC2A, TEENSY_PIN_ENC2B);
    encoders[2] = new Encoder(TEENSY_PIN_ENC3A, TEENSY_PIN_ENC3B);
    encoders[3] = new Encoder(TEENSY_PIN_ENC4A, TEENSY_PIN_ENC4B);
    encoders[4] = new Encoder(TEENSY_PIN_ENC5A, TEENSY_PIN_ENC5B);
    encoders[5] = new Encoder(TEENSY_PIN_ENC6A, TEENSY_PIN_ENC6B);
}

void RobotHal::updateEncoders() {
    /// Read Encoder counts
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        _status.sensors.encoders.previous[n] = _status.sensors.encoders.current[n];
        _status.sensors.encoders.current[n] = encoders[n]->read();
    }
}

void RobotHal::updateSensors() {
    updateTOFSensors();
    updateIMU();
}

void RobotHal::updateIMU() {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    _status.updateOrientation(orientationData);
}