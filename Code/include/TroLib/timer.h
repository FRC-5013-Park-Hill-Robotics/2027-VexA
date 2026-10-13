#include "api.h"

#ifndef TIMER_H
#define TIMER_H

namespace TroLib{
    class timer{
    public:
        timer();
        void reset();
        float get_time();
        bool time_has_passed(float time);

        void set_timer(float time);
        bool timer_finnished();

    private:
        float mStartTime;
        float mTimerSetFor;
    };
}

#endif