#include "averageOverTime.h"

using namespace TroLib;

averageOverTime::averageOverTime(float timeWindow) 
: mTimeWindow(timeWindow) {

}

void averageOverTime::addMeasurement(float timestamp, float value) {
    mMeasurements.push_back(std::make_pair(timestamp, value));
}

float averageOverTime::getAverage(float currentTime) {
    float sum = 0.0f;
    int count = 0;

    for (const auto& measurement : mMeasurements) {
        if (currentTime - measurement.first < mTimeWindow) {
            sum += measurement.second;
            count++;
        }
    }

    return count > 0 ? sum / count : 0.0f;
}