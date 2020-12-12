#include "toy_grabber.h"

#include "jaws.h"
#include "lifter.h"

ToyGrabber::ToyGrabber(uint8_t jawsPin, uint8_t lifterPin) : _jaws(jawsPin), _lifter(lifterPin) {
}

ToyGrabber::~ToyGrabber() {
}

void ToyGrabber::update() {
    switch(_currentState) {
        case ToyGrabber::State::WAIT:
            break;

        case ToyGrabber::State::GRABBING:
            break;

        case ToyGrabber::State::LIFTING:
            break;

        case ToyGrabber::State::LOWERING:
            break;

        case ToyGrabber::State::OPENING:
            break;
    }
}

void ToyGrabber::requestState(ToyGrabber::State requestedState) {
    _requestedState = requestedState;
}

ToyGrabber::State ToyGrabber::getState() {
    return _currentState;
}