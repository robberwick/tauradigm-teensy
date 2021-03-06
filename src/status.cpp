#include "status.h"

#include <Adafruit_Sensor.h>
#include <Arduino.h>

#include "config.h"

float Status::getBatteryVoltage() {
    // TODO - debounce this
    //reads ADC, interprets it and
    //returns battery voltage as a float
    float adcReading, voltage;
    //AnalogRead returns 10bit fraction of Vdd
    adcReading = analogRead(TEENSY_PIN_BATT_SENSE) * 3.3 / 1023.0;

    //ADC reads battery via a potential divider of 33k and 10k
    //but they're wrong/out of spec ((33+10)/10 = 4.3)
    voltage = adcReading * 3.71;
    return voltage;
}

bool Status::batteryIsLow() {
    return getBatteryVoltage() < _minBatVoltage;
}

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