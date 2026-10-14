#include "claw.h"

using namespace TroLib;

#include "constants.h"
#include "utility.h"

claw::claw(std::uint8_t distancePort, std::uint8_t pistonPort)
: mDistance(distancePort)
, mPiston(pistonPort)
, mTimer()
, mMode(null)
{
}

void claw::update()
{
    float current_time = pros::millis();
    distanceMessurements.push_back(std::make_pair(current_time, mDistance.getDistance()));
    
    // Remove data outside the interpolation window
    while (!distanceMessurements.empty() && distanceMessurements.front().first < current_time - claw_const::interpolation_window) {
        distanceMessurements.erase(distanceMessurements.begin());
    }

    //Predict where the pin will be in *interpolation_window* milliseconds
    float predictedDistance = linearInterpolator(distanceMessurements, claw_const::interpolation_window);

    if(mMode == grab){
        if(mTimer.timer_finnished()){
            mMode == null;
        }
		if(predictedDistance < claw_const::tolerance){
            activate(false);
		}
	}
}

void TroLib::claw::autograb(float timeOut)
{
    mTimer.set_timer(timeOut);
    mMode == grab;
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
