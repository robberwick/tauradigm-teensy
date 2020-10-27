#include "robot_hal.h"

#include "config.h"
#include "types.h"
#include "utils.h"

RobotHal::RobotHal(Status &status) : _status(status){};

void RobotHal::initialiseMotors() {
    // attach motors and set to dead stop
    motors.left.attach(TEENSY_PIN_DRIVE_LEFT);
    motors.right.attach(TEENSY_PIN_DRIVE_RIGHT);
    stopMotors();
};

void RobotHal::stopMotors() {
    setMotorSpeeds(_deadStop);
}

void RobotHal::setMotorSpeeds(Speeds requestedMotorSpeeds) {
    Speeds commandMotorSpeeds, targetMotorSpeeds;

    //convert -100 - +100 percentage speed command into mm/sec
    // for autonomous control we could revert back to using full scale
    // but for manual control, and for testing speedcontrol precision
    // better to start with limiting to lower speeds
    float maxspeed_mm_per_sec = 1000;  //max acheivable is ~3200
    targetMotorSpeeds.right = requestedMotorSpeeds.right * maxspeed_mm_per_sec / 100;
    targetMotorSpeeds.left = requestedMotorSpeeds.left * maxspeed_mm_per_sec / 100;

    //convert speed commands into predicted power
    // otherwise known as feedforward. We can do feedforward
    // and/or PID speed control. both is better but either
    // alone gives functional results

    //get predicted motor powers from feedforward
    commandMotorSpeeds = feedForward(targetMotorSpeeds);

    // check if the command speed has been close to zero for a while
    _status.averageSpeed = 0.5 * _status.averageSpeed + 0.5 * (abs(commandMotorSpeeds.left) + abs(commandMotorSpeeds.right));

    //if its been zero for a while, just stop, else work out the PID modified speeds
    if (_status.averageSpeed < -_minSpeed) {
        commandMotorSpeeds = _deadStop;
    } else {
        //apply PID to motor powers based on deviation from target speed
        commandMotorSpeeds = PID(targetMotorSpeeds, commandMotorSpeeds);
    }

    // save the current commanded motor speeds to the status obj

    motors.left.writeMicroseconds(map(commandMotorSpeeds.left, -100, 100, 1000, 2000));
    motors.right.writeMicroseconds(map(commandMotorSpeeds.right * -1, -100, 100, 1000, 2000));
}

Speeds RobotHal::feedForward(Speeds targetSpeeds) {
    // takes two speed commands in mm/sec
    // returns predicted motor power -100 to +100%
    //inputs and outputs both Speed structs

    struct Speeds commandSpeeds;

    float minTurnPower = 4;       //determined from practical testing
    float minForwardPower = 5;    //same
    float powerCoefficient = 50;  //same
    float turnThreshold = 100;    //units: mm/sec. arbitary, value.
    // using the turnThreshold does create a discontinuity when transitioning
    // from mostly straight ahead to a slight turn but then the two moves
    // do need different power outputs. maybe linear interpolation between
    // the two would be better?

    // since there's a min power needed to move (as defined above)
    // first check if we're trying to move
    if (targetSpeeds.left != 0 and targetSpeeds.right != 0) {
        //then check if we're trying to turn or not, i.e. left and right speeds different
        if (abs(targetSpeeds.right - targetSpeeds.left) > turnThreshold) {
            //then predict power needed to acheive that speed. formule derived from curve fitting experimental results
            float turnComponent = sgn(targetSpeeds.right - targetSpeeds.left) * (abs(targetSpeeds.right - targetSpeeds.left) / powerCoefficient + minTurnPower);
            float forwardComponent = (targetSpeeds.right + targetSpeeds.left) / 2 / powerCoefficient;
            commandSpeeds.right = turnComponent + forwardComponent;
            commandSpeeds.left = -turnComponent + forwardComponent;
        } else {
            //a different formula is best fit for going straight
            commandSpeeds.right = sgn(targetSpeeds.right) * (abs(targetSpeeds.right) / powerCoefficient + minForwardPower);
            commandSpeeds.left = sgn(targetSpeeds.left) * (abs(targetSpeeds.left) / powerCoefficient + minForwardPower);
        }
    } else {
        //if we're not trying to move, turn the motors off
        commandSpeeds.right = 0;
        commandSpeeds.left = 0;
    }
    return commandSpeeds;
}

Speeds RobotHal::PID(Speeds targetSpeeds, Speeds commandSpeeds) {
    // apply PID
    // takes two speed commands in -100 to +100 and two
    // target speeds in mm/sec
    // uses sensor feedback to correct for errors
    // returns motor power -100 to +100%
    //inputs and outputs all Speed structs

    // or at the moment, just proportional
    //. i.e power percentage proporational to difference
    // between desired speed and current actual wheel speed
    static long lastLoopTime;
    float loopTime = (millis() - lastLoopTime) / 1000.0;  // divide by 1000 converts to seconds.
    lastLoopTime = millis();

    //work out target turn rate
    float targetTurnRate = (targetSpeeds.left - targetSpeeds.right) / _trackWidth;

    //compare old and latest encoder readings to see how much each wheel has rotated
    //speed is distance/time and should be a float in mm/sec
    Speeds travel = getWheelTravel();
    Speeds actualMotorSpeeds;
    actualMotorSpeeds.right = travel.right / loopTime;
    actualMotorSpeeds.left = travel.left / loopTime;

    //work out actual turn rate
    // TODO - do this in the status obj? How to get loop time?
    float actualTurnRate = wrapTwoPi(_status.orientation.x - _status.previousOrientation.x) / loopTime;

    // do actual Proportional calc.
    //speed error is target - actual.
    float fwdKp = 0.01;  //ie. how much power to use for a given speed error
    commandSpeeds.left += fwdKp * (targetSpeeds.left - actualMotorSpeeds.left);
    commandSpeeds.right += fwdKp * (targetSpeeds.right + actualMotorSpeeds.right);
    float turnKp = 2;
    float steeringCorrection = turnKp * (targetTurnRate - actualTurnRate);
    commandSpeeds.left += steeringCorrection;
    commandSpeeds.right -= steeringCorrection;

    //constrain output
    float max_power = 65;
    commandSpeeds.left = max(min(commandSpeeds.left, max_power), -max_power);
    commandSpeeds.right = max(min(commandSpeeds.right, max_power), -max_power);

    return commandSpeeds;
}

Speeds RobotHal::getWheelTravel() {
    // Uses minimum encoder reading to estimate actual travel speed.
    // returns a speed struct of wheel travel in mm

    //compare old and latest encoder readings to see how much each wheel has rotated
    float wheelTravel[NUM_ENCODERS];
    for (u_int8_t n = 0; n < NUM_ENCODERS; n++) {
        wheelTravel[n] = ((float)(_status.sensors.encoders.current[n] - _status.sensors.encoders.previous[n])) * _travelPerEncoderCount;
    }
    //most representative speed assumed to be slowest wheel
    //#0, #1 & #3 is left,  #3, #4 & #5 is right
    //#0 is front left     #3 is front right
    //#1 is rear left      #4 is middle right
    //#2 is middle left    #5 is rear right
    Speeds travel;
    travel.right = minMagnitude(wheelTravel[3], wheelTravel[4], wheelTravel[5]);
    travel.left = minMagnitude(wheelTravel[0], wheelTravel[1], wheelTravel[2]);
    return travel;
}

float RobotHal::getDistanceTravelled() {
    //returns the average distance travelled by right and left wheels, in mm
    Speeds travel = getWheelTravel();
    return (travel.left - travel.right) / 2;
}