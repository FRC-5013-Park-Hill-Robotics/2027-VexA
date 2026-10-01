#include "distanceSensor.h"

distanceSensor::distanceSensor(std::uint8_t port, float tolerance) 
: mDistanceSensor(port)
, mTolerance(tolerance) {
    
}

float distanceSensor::getDistance(){
    return mDistanceSensor.get();
}

bool distanceSensor::getDigital(){
    return mDistanceSensor.get() < mTolerance;
}