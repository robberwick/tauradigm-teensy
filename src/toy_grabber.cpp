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
}

void ToyGrabber::update() {
    _jaws.update();
    _lifter.update();
    switch (_currentState) {
        case ToyGrabber::State::WAIT:
            if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                _currentState = ToyGrabber::State::GRABBING;
                _jaws.open();
            } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                _currentState = ToyGrabber::State::LOWERING;
                _lifter.down();
            }

            break;
        case ToyGrabber::State::OPENING:
            if (_jaws.getStatus() == Jaws::Status::OPEN) {
                if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                    _currentState = ToyGrabber::State::LOWERING;
                    _lifter.down();
                } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                    _currentState = ToyGrabber::State::WAIT;
                }
            }
            break;

        case ToyGrabber::State::LOWERING:
            if (_lifter.getStatus() == Lifter::Status::DOWN) {
                if (_requestedCommand == ToyGrabber::Command::PICKUP) {
                    _currentState = ToyGrabber::State::GRABBING;
                    _jaws.close();
                } else if (_requestedCommand == ToyGrabber::Command::DEPOSIT) {
                    _currentState = ToyGrabber::State::OPENING;
                    _jaws.open();
                }
            }
            break;

        case ToyGrabber::State::GRABBING:
            if (_jaws.getStatus() == Jaws::Status::CLOSED) {
                _lifter.up();
                _currentState = ToyGrabber::State::LIFTING;
            }
            break;
        case ToyGrabber::State::LIFTING:
            _currentState = ToyGrabber::State::WAIT;
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