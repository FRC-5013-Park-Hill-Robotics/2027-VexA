#include "api.h"

#include "distanceSensor.h"
#include "piston.h"
#include "timer.h"

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
        double getDistance();
        double getInterpolationDist();
        std::string getMode();


    private:
        std::vector<std::pair<float, float>> distanceMessurements; //Distance, timestamp

        distanceSensor mDistance;
        piston mPiston;
        timer mTimer;
        mode mMode;
    };
}

#endif