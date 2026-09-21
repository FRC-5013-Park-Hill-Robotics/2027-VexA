#include "piston.h"
#include "constants.h"

piston::piston(std::uint8_t port)
: mPiston(port, false)
{
}

void piston::toggle(){
    mPiston.toggle();
}

void piston::activate(bool state)
{
    if(state == true){
        mPiston.extend();
    } 
    else{
        mPiston.retract();
    }
}
