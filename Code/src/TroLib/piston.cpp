#include "piston.h"
#include "constants.h"

using namespace TroLib;

piston::piston(std::uint8_t port)
: mPiston(port, false)
, mState(false)
{
}

void piston::toggle(){
    mPiston.toggle();
    mState = !mState;
}

void piston::activate(bool state)
{
    if(state == true){
        mPiston.extend();
    } 
    else{
        mPiston.retract();
    }
    mState = state;
}

bool piston::getState(){
    return mState;
}
