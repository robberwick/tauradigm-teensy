
#include <Arduino.h>
#include <Servo.h>
#include "feed_the_fish.h"

Servo ballLoader;

void challengeSetup(){
    ballLoader.attach(ballLoaderServoPin);
    pinMode(solenoidPin, OUTPUT);
    digitalWrite(solenoidPin, HIGH);
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
        ballLoader.writeMicroseconds(ballLoaderDefault);
        delay(15000);
        ballLoader.writeMicroseconds(ballLoaderDefault+5);
        delay(5000);
        fireSolenoid(firingTime);
        ballLoaderEase(ballLoaderDefault, ballLoaderLoadRowOne, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderLoadRowOne);
        delay(ballSettlingTime);
        ballLoaderEase(ballLoaderLoadRowOne, ballLoaderDefault, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderDefault);
        delay(ballSettlingTime);
        fireSolenoid(firingTime);
        ballLoaderEase(ballLoaderDefault, ballLoaderLoadRowTwo, ballLoadingStep);
        ballLoader.writeMicroseconds(ballLoaderLoadRowTwo);
        delay(ballSettlingTime);
        fireSolenoid(firingTime);
    }
}

void fireSolenoid(int duration)
{
    digitalWrite(solenoidPin, LOW);
    delay(duration);
    digitalWrite(solenoidPin, HIGH);
    delay(solenoidResetTime);
}