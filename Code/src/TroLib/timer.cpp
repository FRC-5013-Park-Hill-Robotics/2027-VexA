#include "timer.h"

timer::timer()
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