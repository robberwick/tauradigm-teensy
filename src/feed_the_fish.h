

#include "config.h"
#include "utils.h"

const int solenoidFire = 1000;
const int solenoidReset = 2000;
const int firingTime = 23;
const int solenoidResetTime = 1000;
const int ballLoaderDefault = 1350;
const int ballLoaderLoadRowOne = 1600;
const int ballLoaderLoadRowTwo = 1850;
const int ballSettlingTime = 800;
const int ballLoadingStep = 25;
const int solenoidPin = TEENSY_PIN_LH_BALL_ESC;
const int ballLoaderServoPin = TEENSY_PIN_SERVO;

void challengeSetup();
void challengeHandler();
void ballLoaderEase(int firstValue, int secondValue, int step);
void fireSolenoid(int duration);