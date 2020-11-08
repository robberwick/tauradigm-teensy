#include "navigation.h"

float headingToWaypoint(Pose target, Pose current){
    float dx, dy, relativeHeading;
    dx = target.x-current.x;
    dy = target.y-current.y;
    if (dy != 0) {
        relativeHeading = (float) atan2(dy, dx);
    } else {
        if (dx>0) {
            relativeHeading = 0;    
        } else {
            relativeHeading = M_PI;
        }
    }
    relativeHeading = wrapTwoPi(relativeHeading - current.heading);

    return relativeHeading;
}