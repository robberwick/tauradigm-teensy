#include <Arduino.h>

#include "toy_grabber.h"

ToyGrabber toyGrabber(A1, A2);

void setup() {
    Serial.begin(115200);
    // while (!Serial) {
    // };
    toyGrabber.begin();
    delay(2000);
}

void loop() {
    if (toyGrabber.getState() == ToyGrabber::State::WAIT) {
        delay(500);
        toyGrabber.pickup();
    }
    toyGrabber.update();
}
