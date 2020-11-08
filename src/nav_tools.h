
#ifndef _NAV__H_
#define _NAV__H_

#include "utils.h"
#include "types.h"

float headingToWaypoint(Pose target, Pose current);
float distanceToWaypoint(Pose target, Pose current);

#endif //_NAV_H