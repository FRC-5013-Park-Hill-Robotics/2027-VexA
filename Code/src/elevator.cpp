#include "elevator.h"
#include "constants.h"
#include "api.h"

elevator::elevator(std::uint8_t left_port, std::uint8_t right_port)
: elevatorMotorLeft(left_port, elevator_const::gearset, elevator_const::units)
, elevatorMotorRight(right_port, elevator_const::gearset, elevator_const::units)
, mSlow(false)
, mMode(manual)
, mCurrentPosition(0)
, mTargetPosition(0)
{
    mRunning = 0;
    elevatorMotorLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    elevatorMotorRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    elevatorMotorLeft.set_reversed(false);
    elevatorMotorRight.set_reversed(true);
}

void elevator::update()
{
    if(mMode == automatic){
        mCurrentPosition = (elevatorMotorLeft.get_position() + elevatorMotorRight.get_position()) / 2.0;
        
        int error = mTargetPosition - mCurrentPosition;

        if(std::abs(error) < 5){ // If within 5 degrees of target
            elevatorMotorLeft.move_velocity(0);
            elevatorMotorRight.move_velocity(0);
            mRunning = 0;
        } else {
            int direction = (error > 0) ? 1 : -1;
            elevatorMotorLeft.move_velocity(direction * elevator_const::slow_speed);
            elevatorMotorRight.move_velocity(direction * elevator_const::slow_speed);
        }
    }
}

void elevator::run(int direction)
{
    mMode = manual;
    this->mRunning = direction;
    if(direction == 0){
        elevatorMotorLeft.move_velocity(0);
        elevatorMotorRight.move_velocity(0);
    }
    else if(direction == 1){
        //elevatorMotorLeft.move_velocity(-500);
        //elevatorMotorRight.move_velocity(-500);
        if(mSlow){
            elevatorMotorLeft.move_velocity(elevator_const::slow_speed);
            elevatorMotorRight.move_velocity(elevator_const::slow_speed);
        } else{
            elevatorMotorLeft.move_velocity(elevator_const::speed);
            elevatorMotorRight.move_velocity(elevator_const::speed);
        }
    }
    else if(direction == -1){
        
        if(mSlow){
            elevatorMotorLeft.move_velocity(-elevator_const::slow_speed);
            elevatorMotorRight.move_velocity(-elevator_const::slow_speed);
        } else{
            elevatorMotorLeft.move_velocity(-elevator_const::speed);
            elevatorMotorRight.move_velocity(-elevator_const::speed);
        }
    }
}

int elevator::getRunning()
{
    return mRunning;
}

void elevator::slow(bool pSlow)
{
    mSlow = pSlow;
}

void elevator::goToPosition(float position)
{
    mMode = automatic;
    mTargetPosition = position;
}

void elevator::zeroPosition()
{
}

float elevator::getCurrentPosition()
{
    return mCurrentPosition;
}

bool elevator::isAtPosition()
{
    return false;
}

elevator::mode elevator::getMode()
{
    return mMode;
}
