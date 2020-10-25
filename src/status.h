#ifndef _STATUS__H_
#define _STATUS__H_

#include <stdint.h>
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

class Status {
   public:
    SensorData sensors;
    Speeds speed;
    Pose pose;
    uint32_t missedMotorMessageCount = 0;

    float getBatteryVoltage();
    boolean batteryIsLow();

   private:
    float _minBatVoltage = 11.1;
};

#endif  //_STATUS__H_