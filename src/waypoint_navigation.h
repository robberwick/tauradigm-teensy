#ifndef _WAYPOINT__H_
#define _WAYPOINT__H_

#include "types.h"
#include <stdint.h>
#include "nav_tools.h"
#include "PID.h"

class WaypointNav {
    public:
      uint8_t currentWaypoint;
      PID headingPID;
      PID speedPID;
      float minSpeed;
      float maxSpeed;
      float maxCorrection;
      float positionTolerance;
      float distanceToGo;
      float previousHeadingError;
      Pose poseOffset; //offset between robot coordiante system and waypoint coordinate system
      Pose currentPose; //Pose within waypoint navigation coordinate system
      Pose targetWaypoint;
      bool finished;
      Pose *route;
      uint8_t numOfWaypoints;
      uint8_t nextWaypoint(uint8_t currentPoint, Pose currentPosition);
      Speeds update(Pose currentPosition,  float previousOrientation, float loopTime);
      //waypoint route variable handling as recommended by 
      // https://stackoverflow.com/questions/37074763/arduino-class-array-member/50924520
      WaypointNav(Pose* RouteToNav, uint8_t NumPoints);
      ~WaypointNav();
      void updateCurrentPose(Pose currentPosition);
};
#endif //_WAYPOINT__H_