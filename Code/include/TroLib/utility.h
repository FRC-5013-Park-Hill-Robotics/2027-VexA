#include "api.h"

#ifndef UTILITY_H
#define UTILITY_H

namespace TroLib{
    float linearInterpolator(std::vector<std::pair<float,float>> points, float interpolateTime); //(time, measurement), time   
}

#endif