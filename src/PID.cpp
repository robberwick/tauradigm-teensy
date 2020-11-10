
#include "PID.h"

float PID::update(float targetValue, float currentValue, float previousValue, float timeInterval){
    // uses proportional, differential and integral gains
    // to calcualte the response for a given error from target
    // values floats
    float error = currentValue - targetValue;
    if (piWrapping) {
        error = wrapTwoPi(error);
    }
    float change = currentValue - previousValue;
    cumulativeError += error;
    float output = pGain * error - dGain * change / timeInterval + iGain * cumulativeError;
    return output;
};