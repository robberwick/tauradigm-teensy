
#include <Arduino.h>
#include <Servo.h>
#include "feed_the_fish.h"

Servo ballLoader;

void challengeSetup(){
    ballLoader.attach(ballLoaderServoPin);
    pinMode(solenoidPin, OUTPUT);
    digitalWrite(solenoidPin, LOW);
}

void ballLoaderEase(int firstValue, int secondValue, int step){
    int currentValue = firstValue;
    int maxLoops =  (int)abs(((secondValue - firstValue)) / step);
    step = sgn(secondValue - firstValue) * step;
    int loops = 0;
    while (loops < maxLoops){
        currentValue += step;
        ballLoader.writeMicroseconds(currentValue);
        loops += 1;
        delay(20);
    }
}

void challengeHandler(){
    while (true){
        ballLoader.writeMicroseconds(ballLoaderDefault+5);
        delay(3000);
        fireSolenoid(firingTime);
        ballLoaderEase(ballLoaderDefault, ballLoaderMidTravel, ballMidTravelStep);
        ballLoaderEase(ballLoaderMidTravel, ballLoaderLoadRowOne, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderLoadRowOne);
        delay(ballSettlingTime);
        ballLoaderEase(ballLoaderLoadRowOne, ballLoaderDefault, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderDefault);
        delay(ballSettlingTime);
        fireSolenoid(firingTime);
        ballLoaderEase(ballLoaderDefault, ballLoaderMidTravel, ballMidTravelStep);
        ballLoaderEase(ballLoaderMidTravel, ballLoaderLoadRowTwo, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderLoadRowTwo);
        delay(ballSettlingTime);
        fireSolenoid(firingTime);
        delay(11000);
        ballLoaderEase(ballLoaderLoadRowTwo, ballLoaderLoadRowOne, ballReloadingStep);
        ballLoader.writeMicroseconds(ballLoaderLoadRowOne);
        delay(11000);
        ballLoaderEase(ballLoaderLoadRowOne, ballLoaderDefault, ballReloadingStep);
        ballLoader.writeMicroseconds(ballLoaderDefault);
        delay(11000);
    }
}

void fireSolenoid(float duration)
{//swapped high and low
    digitalWrite(solenoidPin, HIGH);
    delay(duration);
    digitalWrite(solenoidPin, LOW);
    delay(solenoidResetTime);
}