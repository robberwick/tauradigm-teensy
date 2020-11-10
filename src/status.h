#ifndef _STATUS__H_
#define _STATUS__H_

#include <Adafruit_Sensor.h>
#include <stdint.h>

#include "types.h"

class Status {
   public:
    ActivationStatus activation;
    SensorData sensors;
    struct
    {
        Speeds currentSpeed;
        Speeds requestedSpeed;
        float averageSpeed;
    } speeds;
    float averageSpeed;
    Pose pose;
    float headingOffset;
    Pose targetPose;
    uint8_t waypointNumber;
    uint8_t numWaypoints;
    float wayHead;
    float wayDist;
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