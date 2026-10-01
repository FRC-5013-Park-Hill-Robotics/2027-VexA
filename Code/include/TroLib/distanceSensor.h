#include "api.h"

#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

class distanceSensor{
public:
    distanceSensor(std::uint8_t port, float tolerance);
    float getDistance();
    bool getDigital();
private:
    pros::Distance mDistanceSensor;
    float mTolerance;
};

#endif

