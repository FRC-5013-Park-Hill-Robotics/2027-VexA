#include "api.h"

#ifndef PISTON_H
#define PISTON_H

namespace TroLib{
class piston{
    public:
        piston(std::uint8_t port);
        void toggle();
        void activate(bool state);
        bool getState();
    private:
        pros::adi::Pneumatics mPiston;
        bool mState;
    };
}

#endif