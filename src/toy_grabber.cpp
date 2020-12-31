#include "toy_grabber.h"

#include "jaws.h"
#include "lifter.h"

ToyGrabber::ToyGrabber(uint8_t jawsPin, uint8_t lifterPin) : _jaws(jawsPin), _lifter(lifterPin) {
}

ToyGrabber::~ToyGrabber() {
}

void ToyGrabber::begin() {
    _lifter.begin();
    _lifter.down();

    _jaws.begin();
    _jaws.open();
    while ((_jaws.getStatus() != Jaws::Status::OPEN) && _lifter.getStatus() != Lifter::Status::DOWN) {
        update();
    }
}

void ToyGrabber::update() {
    _jaws.update();
    _lifter.update();
    switch (_currentState) {
        case ToyGrabber::State::WAIT:
            if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                _currentState = ToyGrabber::State::OPENING;

            } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                _currentState = ToyGrabber::State::LOWERING;
            }

            break;
        case ToyGrabber::State::OPENING:
            if (_jaws.getStatus() == Jaws::Status::OPEN) {
                if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                    _currentState = ToyGrabber::State::LOWERING;

                } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                    _currentState = ToyGrabber::State::WAIT;
                }
            } else {
                _jaws.open();
            }
            break;

        case ToyGrabber::State::LOWERING:
            if (_lifter.getStatus() == Lifter::Status::DOWN) {
                if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                    _currentState = ToyGrabber::State::GRABBING;

                } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                    _currentState = ToyGrabber::State::OPENING;
                }
            } else {
                _lifter.down();
            }
            break;

        case ToyGrabber::State::GRABBING:
            if (_jaws.getStatus() == Jaws::Status::CLOSED) {
                _currentState = ToyGrabber::State::LIFTING;
            } else {
                _jaws.close();
            }
            break;
        case ToyGrabber::State::LIFTING:
            if (_lifter.getStatus() == Lifter::Status::UP) {
                if (_requestedCommand == Command::PICKUP) {
                    _requestedCommand = Command::STOP;
                }
                _currentState = ToyGrabber::State::WAIT;
            } else {
                _lifter.up();
            }
            break;
    }
}

ToyGrabber::State ToyGrabber::getState() {
    return _currentState;
}

void ToyGrabber::pickup() {
    _requestedCommand = ToyGrabber::Command::PICKUP;
}

void ToyGrabber::deposit() {
    // TODO - what do we want to do here? do we care?
    _requestedCommand = ToyGrabber::Command::DEPOSIT;
}