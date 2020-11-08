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

float distanceToWaypoint(Pose target, Pose current){
    //returns distance 'as the crow flies' to the target pose 
    float distance;
    //hypotenuse of dx, dy triangle gives distance, using h^2=x^2+y^2
    distance = sqrt(powf((target.x-current.x),2) + powf((target.y-current.y),2));
//    display.println(" ");
//    display.printf("distance: %2.2f", distance);
    return distance;
}