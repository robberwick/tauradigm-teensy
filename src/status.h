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
    struct
    {
        float currentV;
        float averageV;
        bool isLow;
    } battery;
    
    Pose pose;
    uint32_t missedMotorMessageCount = 0;

    void incrementMissedMotorCount();
    void resetMissedMotorCount();
    bool motorMessageCommsDown();

    Pose updatePose(float heading, float distanceTravelled);

    void updateOrientation(sensors_event_t orientationData);

    OrientationReading orientation;
    OrientationReading previousOrientation;

    GitData git;

};

#endif  //_STATUS__H_