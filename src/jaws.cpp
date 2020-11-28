#include "Jaws.h"

#include <Servo.h>

#include "Arduino.h"

Jaws::Jaws(uint8_t pin) {
    _pin = pin;
}

void Jaws::begin() {
    pinMode(_pin, OUTPUT);
    _servo.attach(_pin);
}

void Jaws::open() {
    if (_status == Status::CLOSED || _status == Status::CLOSING) {
        _status = Status::OPENING;
        _startMillis = millis();
    }
    update();
}

void Jaws::close() {
    if (_status == Status::OPEN || _status == Status::OPENING) {
        _status = Status::CLOSING;
        _startMillis = millis();
    }
    update();
}

void Jaws::update() {
    uint32_t currMillis = millis();

    switch (_status) {
        case Status::OPEN:
            break;
        case Status::OPENING:
            if ((currMillis - _startMillis) > _servoDelay) {
                _status = Status::OPEN;
            }
            break;
        case Status::CLOSING:
            if ((currMillis - _startMillis) > _servoDelay) {
                _status = Status::CLOSED;
            }
            break;
        case Status::CLOSED:
            break;
    }
}

Jaws::Status Jaws::getStatus() { return _status; }