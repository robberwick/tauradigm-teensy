#ifndef _TYPES__H_
#define _TYPES__H_

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
};

struct ActivationStatus {
    bool tofSensors[8] = {false, false, false, false, false, false, false, false};
};

struct Speeds {
    float left = 0.0;
    float right = 0.0;
};

struct GitData {
    char branch[21] = GIT_BRANCH;
    char commit[8] = GIT_REV;
};

#endif  // _TYPES__H_