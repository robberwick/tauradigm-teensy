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
    if (_status == Status::DOWN || _status == Status::LOWERING) {
        _status = Status::LIFTING;
        _startMillis = millis();
    }
    update();
}

void Lifter::down() {
    if (_status == Status::UP || _status == Status::LIFTING) {
        _status = Status::LOWERING;
        _startMillis = millis();
    }
    update();
}

void Lifter::update() {
    static uint32_t currMillis = millis();
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