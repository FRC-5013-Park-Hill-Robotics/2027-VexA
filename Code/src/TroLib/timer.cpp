#include "timer.h"

using namespace TroLib;

timer::timer()
: mTimerSetFor(1000000)
{
    reset();
}

void timer::reset()
{   
    mStartTime = pros::millis();
}

float timer::get_time()
{
    return (pros::millis() - mStartTime); // Convert milliseconds to seconds
}

bool timer::time_has_passed(float time)
{
    return (get_time() >= time); // Convert milliseconds to seconds
}

void timer::set_timer(float time)
{
    mTimerSetFor = time;
    reset();
}

bool timer::timer_finnished()
{
    return time_has_passed(mTimerSetFor);
}
