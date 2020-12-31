#ifndef Lifter_h
#define Lifter_h

#include <Servo.h>

#include "Arduino.h"

class Lifter {
   public:
    enum Status {
        UNSET,
        UP,
        DOWN,
        LIFTING,
        LOWERING
    };
    Lifter(uint8_t pin);
    void begin();
    void up();
    void down();
    void update();
    Status getStatus();

   private:
    uint8_t _pin;
    Servo _servo;
    uint16_t _degUp = 55;
    uint16_t _degDown = 123;
    Status _status;
    uint32_t _startMillis;
    uint32_t _servoDelay = 100;  // ms delay to wait for servo to settle after commanding a move
};

#endif