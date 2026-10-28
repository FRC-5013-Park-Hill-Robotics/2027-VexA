#include "claw.h"

using namespace TroLib;

#include "constants.h"
//#include "utility.h"

// float interpolator(std::vector<std::pair<float, float>> points, float interpolateTime)
// {
//     if (points.size() < 2)
//         throw std::invalid_argument("At least two points are required");

//     // Use the two most recent measurements.
//     const auto& p1 = points[points.size() - 2];
//     const auto& p2 = points[points.size() - 1];

//     if (p2.first == p1.first)
//         throw std::invalid_argument("Timestamps must be different");

//     // Calculate the rate of change per unit of time.
//     float slope = (p2.second - p1.second) /
//                   (p2.first - p1.first);

//     // Project the trend to the requested future time.
//     return p2.second + slope * (interpolateTime - p2.first);
// }

claw::claw(std::uint8_t distancePort, std::uint8_t pistonPort)
: mDistance(distancePort)
, mPiston(pistonPort)
, mTimer()
, mMode(null)
{
}

void claw::update()
{
    //float current_time = pros::millis();
    //distanceMessurements.push_back(std::make_pair(current_time, mDistance.getDistance()));
    
    // Remove data outside the interpolation window
    // while (!distanceMessurements.empty() && distanceMessurements.front().first < current_time - claw_const::interpolation_window) {
    //     distanceMessurements.erase(distanceMessurements.begin());
    // }

    //Predict where the pin will be in *interpolation_window* milliseconds
    float predictedDistance = mDistance.getDistance(); //interpolator(distanceMessurements, claw_const::interpolation_window);

    if(mMode == grab){
        // if(mTimer.timer_finnished()){
        //     mMode == null;
        // }
		if(predictedDistance < claw_const::tolerance){
            activate(false);
            //mMode == null;
		}
	}
}

void claw::autograb(float timeOut)
{
    mTimer.set_timer(timeOut);
    mMode == claw::grab;
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

double claw::getDistance()
{
    return mDistance.getDistance();
}

double claw::getInterpolationDist()
{
    return 0;//interpolator(distanceMessurements, claw_const::interpolation_window);
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