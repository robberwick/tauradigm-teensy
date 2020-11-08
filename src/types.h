#ifndef _TYPES__H_
#define _TYPES__H_
#include <Adafruit_Sensor.h>
#include <stdint.h>

struct OrientationReading {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct IMUCalibrationReading {
    uint8_t system = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
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
    struct IMUData {
        IMUCalibrationReading calibration;
        sensors_event_t imuEvent;
    } imu;
};

struct ActivationStatus {
    bool i2c[128];
    bool imu = false;
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

enum controlMode {
    RC,
    AUTO
};

#endif  // _TYPES__H_