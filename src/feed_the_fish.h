

#include "config.h"
#include "utils.h"

const int solenoidFire = 1000;
const int solenoidReset = 2000;
const float firingTime = 20.55;
const int solenoidResetTime = 1000;
const int ballLoaderDefault = 1220;
const int ballLoaderMidTravel = 1300;
const int ballLoaderLoadRowOne = 1580;
const int ballLoaderLoadRowTwo = 1850;
const int ballSettlingTime = 2000;
const int ballLoadingStep = 20;
const int ballMidTravelStep = 5;
const int ballReloadingStep = 5;
const int solenoidPin = TEENSY_PIN_LH_BALL_ESC;
const int ballLoaderServoPin = TEENSY_PIN_SERVO;

void challengeSetup();
void challengeHandler();
void ballLoaderEase(int firstValue, int secondValue, int step);
void fireSolenoid(float duration);