#include <vector>

#ifndef AVERAGE_OVER_TIME_H
#define AVERAGE_OVER_TIME_H

namespace TroLib{
    class averageOverTime {
    public:
        averageOverTime(float timeWindow);
        void addMeasurement(float timestamp, float value);
        float getAverage(float currentTime);

    private:
        float mTimeWindow;
        std::vector<std::pair<float, float>> mMeasurements; // Pair of (timestamp, value)
    };
}

#endif