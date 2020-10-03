/*
Message Type Identifiers

Note: These need to be in sync with the pi side if you want your comms to work :-)
*/
namespace MessageTypes {
    enum {
    MOTOR_COMMAND = 0,
    SENSOR_DATA = 1,
    SYSTEM_STATUS_REQUEST = 2,
    SYSTEM_STATUS_DATA = 3
    };
};
