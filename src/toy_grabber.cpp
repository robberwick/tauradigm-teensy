#include "toy_grabber.h"

#include "jaws.h"
#include "lifter.h"

ToyGrabber::ToyGrabber(uint8_t jawsPin, uint8_t lifterPin) : _jaws(jawsPin), _lifter(lifterPin) {
}

ToyGrabber::~ToyGrabber() {
}

void ToyGrabber::update() {
    _jaws.update();
    _lifter.update();
    switch (_currentState) {
        case ToyGrabber::State::WAIT:
            if (_requestedCommand == ToyGrabber::Command::RUN) {
                _currentState = ToyGrabber::State::GRABBING;
                _jaws.open();
            }
            break;
        case ToyGrabber::State::OPENING:
            if (_jaws.getStatus() == Jaws::Status::OPEN) {
                _currentState = ToyGrabber::State::LOWERING;
                _lifter.down();
            }
            break;

        case ToyGrabber::State::LOWERING:
            if (_lifter.getStatus() == Lifter::Status::DOWN) {
                _currentState = ToyGrabber::State::GRABBING;
                _jaws.close();
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

void ToyGrabber::run() {
    _requestedCommand = ToyGrabber::Command::RUN;
}