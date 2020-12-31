#include "Lifter.h"

#include <Servo.h>

#include "Arduino.h"

Lifter::Lifter(uint8_t pin) {
    _pin = pin;
}

void Lifter::begin() {
    pinMode(_pin, OUTPUT);
    _servo.attach(_pin);
}

void Lifter::up() {
    if (_status != Status::LIFTING) {
        _status = Status::LIFTING;
        _servo.write(_degUp);
        _startMillis = millis();
    }
}

void Lifter::down() {
    if (_status != Status::LOWERING) {
        _status = Status::LOWERING;
        _servo.write(_degDown);
        _startMillis = millis();
    }
}

void Lifter::update() {
    uint32_t currMillis = millis();
    switch (_status) {
        case Status::DOWN:
            break;
        case Status::LIFTING:

            if ((currMillis - _startMillis) > _servoDelay) {
                _status = Status::UP;
            }
            break;
        case Status::LOWERING:
            if ((currMillis - _startMillis) > _servoDelay) {
                _status = Status::DOWN;
            }
            break;
        case Status::UP:
            break;
    }
}

Lifter::Status Lifter::getStatus() {
    return _status;
}