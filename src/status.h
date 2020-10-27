#ifndef _STATUS__H_
#define _STATUS__H_

#include <Adafruit_Sensor.h>
#include <stdint.h>

#include "types.h"

class Status {
   public:
    SensorData sensors;
    Speeds speed;
    float averageSpeed;
    Pose pose;
    uint32_t missedMotorMessageCount = 0;

    float getBatteryVoltage();
    bool batteryIsLow();

    void incrementMissedMotorCount();
    void resetMissedMotorCount();
    bool motorMessageCommsDown();

    Pose updatePose(float heading, float distanceTravelled);

    void updateOrientation(sensors_event_t orientationData);

    OrientationReading orientation;
    OrientationReading previousOrientation;

    GitData git;

   private:
    float _minBatVoltage = 11.1;
};

#endif  //_STATUS__H_