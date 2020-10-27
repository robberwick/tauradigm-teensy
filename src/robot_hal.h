#ifndef _ROBOT_HAL__H_
#define _ROBOT_HAL__H_

#include <Servo.h>

#include "status.h"
#include "types.h"

class RobotHal {
   public:
    RobotHal(Status &status);
    struct Motors {
        Servo left;
        Servo right;
    } motors;

    void initialiseMotors();
    void stopMotors();
    void setMotorSpeeds(Speeds requestedMotorSpeeds);
    Speeds getWheelTravel();
    float getDistanceTravelled();

   private:
    Speeds feedForward(Speeds targetSpeeds);
    Speeds PID(Speeds targetSpeeds, Speeds commandSpeeds);
    Status &_status;
    Speeds _deadStop = {0, 0};
    float _minSpeed = 20;
    float _trackWidth = 136;
    float _travelPerEncoderCount = 0.262;
};

#endif  // _ROBOT_HAL__H_