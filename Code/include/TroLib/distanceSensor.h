#include "api.h"

#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

namespace TroLib{
    class distanceSensor{
    public:
        distanceSensor(std::uint8_t port);
        float getDistance();
    private:
        pros::Distance mDistanceSensor;
        float mLastDistance;
    };
}

#endif