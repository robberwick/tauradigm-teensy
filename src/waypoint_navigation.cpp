#include "waypoint_navigation.h"

WaypointNav::WaypointNav(Pose* RouteToNav, uint8_t NumPoints){
    currentWaypoint = 0;
    headingPID.pGain = 25;
    headingPID.iGain = 0;
    headingPID.dGain = 0;
    headingPID.piWrapping = true;
    speedPID.pGain = 0.25;
    speedPID.iGain = 0;
    speedPID.dGain = 0;
    positionTolerance = 100;
    numOfWaypoints = NumPoints;
    route = (Pose *)malloc(numOfWaypoints * sizeof(Pose));
    for (uint8_t i=0; i<numOfWaypoints; i++) {
     route[i] = RouteToNav[i];
    }
    minSpeed = 20;
    maxSpeed = 25;
    maxCorrection = 30;
    previousHeadingError = 0;
    finished = false;
}

uint8_t WaypointNav::nextWaypoint(uint8_t currentPoint, Pose position){
    uint8_t nextWaypoint;
    nextWaypoint = currentPoint;
    targetWaypoint = route[currentPoint];
    distanceToGo = distanceToWaypoint(targetWaypoint, position); 
    while (distanceToGo < positionTolerance) { 
        nextWaypoint += 1;
        targetWaypoint = route[nextWaypoint];
        distanceToGo = distanceToWaypoint(targetWaypoint, position);
    } 
    return nextWaypoint;
};

Speeds WaypointNav::update(Pose currentPosition, float previousHeading, float loopTime){
    //could really do with timestamped readings, 
    // would eliminate the need for the previous heading and loop time arguments. 
    
    Speeds motorSpeeds;
    //figure out where we're going next
    currentWaypoint = nextWaypoint(currentWaypoint, currentPosition);
    if (currentWaypoint > numOfWaypoints) {
        //if we're here, we've reached the final waypoint
        finished = true;
        motorSpeeds.left = motorSpeeds.right = 0; //would be better to set to deadstop
    } else {
        //otherwise keep navigating
        headingError = headingToWaypoint(targetWaypoint, currentPosition);
        float previousError = headingError - (currentPosition.heading - previousHeading);  //fudgy
        float turnCorrection = headingPID.update(0.0, headingError, previousError, loopTime);
        turnCorrection = min(max(turnCorrection, -maxCorrection), maxCorrection);
        
        //don't bother with speedPID for now
        motorSpeeds.left = minSpeed;
        motorSpeeds.right = minSpeed;
        motorSpeeds.left += turnCorrection;
        motorSpeeds.right -= turnCorrection;
    }
    return motorSpeeds;
}

WaypointNav::~WaypointNav(){
    free(route);
}