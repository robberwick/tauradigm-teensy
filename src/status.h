struct OrientationReading {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Pose {
    float heading =0 ;
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

struct Status {
    float batteryVoltage = 0;
    SensorData sensors;
    Speeds speed;
    Pose pose;
};