#include <Arduino.h>
#include "status.h"
#include "config.h"

float Status::getBatteryVoltage() {
    // TODO - debounce this

    //reads ADC, interprets it and
    //returns battery voltage as a float
    float adcReading, voltage;
    //AnalogRead returns 10bit fraction of Vdd
    // TODO make TEENSY_PIN_BATT_SENSE a class prop set in constructor?
    adcReading = analogRead(TEENSY_PIN_BATT_SENSE) * 3.3 / 1023.0;

    //ADC reads battery via a potential divider of 33k and 10k
    //but they're wrong/out of spec
    voltage = adcReading * (26.9 + 10.0) / 10.0 + 4;
    return voltage;
}

boolean Status::batteryIsLow() {
    return getBatteryVoltage() < _minBatVoltage;
}

void Status::incrementMissedMotorCount() {
    missedMotorMessageCount++;
}

void Status::resetMissedMotorCount() {
    missedMotorMessageCount = 0;
}