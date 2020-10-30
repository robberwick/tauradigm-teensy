#ifndef _UTILS__H_
#define _UTILS__H_

#include <Arduino.h>

// M_PI
// #ifndef M_PI
#define M_PI 3.14159265358979323846
// #endif

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

float minMagnitude(float x, float y, float z);

float wrapTwoPi(float angle);

#endif  // _UTILS__H_