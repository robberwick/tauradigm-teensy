#ifndef Jaws_h
#define Jaws_h

#include <Servo.h>

#include "Arduino.h"

class Jaws {
   public:
    enum Status {
        UNSET,
        OPEN,
        CLOSED,
        OPENING,
        CLOSING
    };
    Jaws(uint8_t pin);
    void begin();
    void open();
    void close();
    void update();
    Status getStatus();

   private:
    uint8_t _pin;
    Servo _servo;
    uint16_t _degOpen = 140;
    uint16_t _degClosed = 90;
    Status _status;
    uint32_t _startMillis;
    uint32_t _servoDelay = 100;
};

#endif