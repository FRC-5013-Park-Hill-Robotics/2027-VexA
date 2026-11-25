#include "api.h"

#include "distanceSensor.h"
#include "piston.h"
#include "timer.h"
#include "averageOverTime.h"

#ifndef CLAW_H
#define CLAW_H

namespace TroLib{
    class claw{
    public:
        enum mode{
            grab,
            null
        };

        claw(std::uint8_t distancePort, std::uint8_t pistonPort);
        void update();
        void autograb(float timeOut);
        void toggle();
        void activate(bool state);
        float getDist();
        std::string getMode();

        float interpolator(float interpolateTime);

    private:
        distanceSensor mDistance;
        piston mPiston;
        timer mTimer;
        mode mMode;

        TroLib::averageOverTime mAverage;

        float acceleration = 0;
        float lastKnownVelocity = 0;

        float time1 = 0;
        float distance1 = 0;
        float time2 = 0;
        float distance2 = 0;
    };
}

#endif