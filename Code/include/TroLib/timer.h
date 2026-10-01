#include "api.h"

#ifndef TIMER_H
#define TIMER_H

class timer{
public:
    timer();
    void reset();
    float get_time();
    bool time_has_passed(float time);

private:
    float mStartTime;
};

#endif