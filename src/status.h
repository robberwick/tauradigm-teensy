#ifndef _STATUS__H_
#define _STATUS__H_

#include <stdint.h>
#include <Adafruit_Sensor.h>

struct OrientationReading {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Pose {
    float heading = 0;
    float x = 0;
    float y = 0;
};

struct SensorData {
    struct EncoderData {
        long previous[6] = {0, 0, 0, 0, 0, 0};
        long current[6] = {0, 0, 0, 0, 0, 0};
    } encoders;
    float tofDistances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    OrientationReading orientation;
};

struct Speeds {
    float left = 0.0;
    float right = 0.0;
};

struct GitData {
    char branch[21] = GIT_BRANCH;
    char commit[8] = GIT_REV;
};

class Status {
   public:
    SensorData sensors;
    Speeds speed;
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