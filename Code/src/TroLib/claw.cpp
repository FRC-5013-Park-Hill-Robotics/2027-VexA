#include "claw.h"

using namespace TroLib;

#include "constants.h"
#include "averageOverTime.h"

claw::claw(std::uint8_t distancePort, std::uint8_t pistonPort)
: mDistance(distancePort)
, mPiston(pistonPort)
, mTimer()
, mMode(null)
, mAverage(claw_const::averaging_window)
{
}

void claw::update()
{
    float currentTime = pros::millis();
    float currentDistance = mDistance.getDistance();
    mAverage.addMeasurement(currentTime, currentDistance);

    distance2 = distance1;
    distance1 = mAverage.getAverage(currentTime);

    time2 = time1;
    time1 = currentTime;

    if(mMode == claw::grab){
        if(interpolator(claw_const::interpolation_window) < claw_const::tolerance){
            activate(false);
            pros::delay(claw_const::interpolation_window + 5);
            mMode = claw::null;
        }
	}
}

void claw::autograb(float timeOut)
{
    mTimer.set_timer(timeOut);
    mMode = claw::grab;
    activate(true);
}

void claw::toggle()
{
    mPiston.toggle();
    mMode = null; //Cancel autoclawing if you're manual controls
}

void claw::activate(bool state)
{
    mPiston.activate(state);
}

float claw::getDist()
{
    return mDistance.getDistance();
}

std::string claw::getMode()
{
    if(mMode == grab){
        return "grab";
    }
    if(mMode == null){
        return "null";
    }
    return "unknown";
}

float claw::interpolator(float interpolateTime)
{
    if (time2 == time1)
        return 1000;

    // Calculate the rate of change per unit of time.
	float dt = time1 - time2;

    float currentVelocity = (distance1 - distance2) /
                  (dt);
	acceleration = (currentVelocity - lastKnownVelocity) / (dt);
	
	lastKnownVelocity = currentVelocity;

    // Project the trend to the requested future time.
	float projectedTime = interpolateTime;

	// Use the kinematic equation: s = s0 + v0*t + 0.5*a*t^2
    return distance1 + currentVelocity * (projectedTime) + 0.5 * acceleration * (projectedTime * projectedTime);
}