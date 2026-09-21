#include "conveyor.h"
#include "constants.h"
#include "api.h"

conveyor::conveyor()
: conveyorMotor(ports_const::conveyor, conveyor_const::gearset, conveyor_const::units)
, mSlow(false)
{
    running = 0;
}

conveyor::conveyor(std::uint8_t port)
: conveyorMotor(port, conveyor_const::gearset, conveyor_const::units)
, mSlow(false)
{
    running = 0;
}

void conveyor::run(int direction)
{
    this->running = direction;
    if(direction == 0){
        conveyorMotor.move_velocity(0);
    }
    else if(direction == 1){
        //conveyorMotor.move_velocity(-500);
        if(mSlow){
            conveyorMotor.move_velocity(-conveyor_const::slow_speed);
        } else{
            conveyorMotor.move_velocity(-conveyor_const::speed); //Spit out
        }
    }
    else if(direction == -1){
        
        if(mSlow){
            conveyorMotor.move_velocity(conveyor_const::slow_speed);
        } else{
            conveyorMotor.move_velocity(conveyor_const::speed); //Spit out
        }
    }
}

int conveyor::getRunning()
{
    return running;
}

void conveyor::slow(bool pSlow)
{
    mSlow = pSlow;
}
