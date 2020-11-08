#include "utils.h"

float minMagnitude(float x, float y, float z) {
    //function to find the smallest (closest to zero) value
    // specifically, find the motor that is spinning slowest
    // which is assumed to be the most representative of robot speed
    float currentMin = x;
    float currentMinMagnitude = abs(x);
    if (abs(y) < currentMinMagnitude) {
        currentMin = y;
        currentMinMagnitude = abs(y);
    }
    if (abs(z) < currentMinMagnitude) {
        currentMin = z;
    }
    return currentMin;
}

float wrapTwoPi(float angle) {
    //wraps an angle to stay within +/-pi
    while (angle > M_PI) angle -= TWO_PI;
    while (angle < -M_PI) angle += TWO_PI;
    return angle;
}

float PID::update(float targetValue, float currentValue, float previousValue, float timeInterval){
    // uses proportional, differential and integral gains
    // to calcualte the response for a given error from target
    // values floats
    float error = currentValue - targetValue;
    float change = currentValue - previousValue;
    cumulativeError += error;
    float output = pGain * error - dGain * change / timeInterval + iGain * cumulativeError;
    return output;
};