#ifndef _ROBOT_HAL__H_
#define _ROBOT_HAL__H_

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "status.h"
#include "types.h"

class RobotHal {
   public:
    RobotHal(Status &status, TwoWire *twi = &Wire);
    struct Motors {
        Servo left;
        Servo right;
    } motors;

    bool initialiseMotors();
    void stopMotors();
    void setMotorSpeeds(Speeds requestedMotorSpeeds);
    Speeds getWheelTravel();
    float getDistanceTravelled();

    void initialiseTOFSensors();
    void updateTOFSensors();

    void initialiseIMU();
    bool calibrateIMU();
    bool restoreCalibrationData();
    void saveCalibrationData();
    bool getIMUEvent();

    void doI2CScan();

   private:
    Speeds feedForward(Speeds targetSpeeds);
    Speeds PID(Speeds targetSpeeds, Speeds commandSpeeds);
    void tcaselect(uint8_t i);

    Status &_status;
    TwoWire *wire;

    Speeds _deadStop = {0, 0};
    float _minSpeed = 20;
    float _trackWidth = 136;
    float _travelPerEncoderCount = 0.262;

    VL53L0X sensor;

    Adafruit_BNO055 bno;
};

#endif  // _ROBOT_HAL__H_