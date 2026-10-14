#include "distanceSensor.h"

#include "utility.h"

using namespace TroLib;

distanceSensor::distanceSensor(std::uint8_t port) 
: mDistanceSensor(port)
, mLastDistance(0) {
}

float distanceSensor::getDistance(){
    double distance = mDistanceSensor.get();
    if(distance < 9999){
        mLastDistance = distance;
    }
    return mLastDistance;
}

