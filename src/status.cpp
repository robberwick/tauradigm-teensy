#include "status.h"

#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include "config.h"

void Status::incrementMissedMotorCount() {
    missedMotorMessageCount++;
}

void Status::resetMissedMotorCount() {
    missedMotorMessageCount = 0;
}

bool Status::motorMessageCommsDown() {
    // TODO Make MAX_MISSED_MOTOR_MESSAGES a class prop?
    return missedMotorMessageCount >= MAX_MISSED_MOTOR_MESSAGES;
}

Pose Status::updatePose(float heading, float distanceTravelled) {
    // determine new position based on heading and distance travelled
    pose.x = pose.x + distanceTravelled * cos(heading);
    pose.y = pose.y + distanceTravelled * sin(heading);
    return pose;
}

void Status::updateOrientation(sensors_event_t orientationData) {
    previousOrientation = orientation;
    orientation.x = radians(orientationData.orientation.x);
    orientation.y = radians(orientationData.orientation.y);
    orientation.z = radians(orientationData.orientation.z);
}