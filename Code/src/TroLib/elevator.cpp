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
, mCurrentSetpointIndex(0)
, mPID(elevator_const::PIDF[0], elevator_const::PIDF[1], elevator_const::PIDF[2], elevator_const::PIDF[3])
{
    mRunning = 0;
    elevatorMotorLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    elevatorMotorRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    elevatorMotorLeft.set_reversed(false);
    elevatorMotorRight.set_reversed(true);

    mPID.setOutputLimits(-1, 1); // Never more that 100% power
}

void elevator::update()
{
    mCurrentPosition = -elevatorMotorLeft.get_position();
    
    if(mMode == automatic){
        float error = mTargetPosition - mCurrentPosition;

        pros::lcd::set_text(1, "Elevator Output: " + std::to_string(error));

        if(std::abs(error) < elevator_const::position_tolerance){ 
            elevatorMotorLeft.move_velocity(0);
            elevatorMotorRight.move_velocity(0);
        } else {
            double output = -mPID.getOutput(mCurrentPosition, mTargetPosition); // Assuming update is called every 20ms
            elevatorMotorLeft.move_velocity(output * elevator_const::speed);
            elevatorMotorRight.move_velocity(output * elevator_const::speed);
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

void elevator::goToSetpoint(int setpoint_index)
{
    if(setpoint_index >= 0 && setpoint_index < std::size(elevator_const::setpoints)){
        mMode = automatic;
        mTargetPosition = elevator_const::setpoints[setpoint_index];
        mCurrentSetpointIndex = setpoint_index;
    }
}

void elevator::incrementSetpoint(int setpoint_index_increment)
{
    int newSetpointIndex = mCurrentSetpointIndex + setpoint_index_increment;
    if(newSetpointIndex >= 0 && newSetpointIndex < std::size(elevator_const::setpoints)){
        mMode = automatic;
        mTargetPosition = elevator_const::setpoints[newSetpointIndex];
        mCurrentSetpointIndex = newSetpointIndex;
    }
}

void elevator::zeroPosition()
{
}

float elevator::getCurrentPosition()
{
    return mCurrentPosition;
}

float elevator::getTargetPosition()
{
    return mTargetPosition;
}

int elevator::getCurrentSetpointIndex()
{
    return mCurrentSetpointIndex;
}

bool elevator::isAtPosition()
{
    float error = mTargetPosition - mCurrentPosition;
    if(std::abs(error) < elevator_const::position_tolerance){ 
        return true;
    }
    return false;
}

elevator::mode elevator::getMode()
{
    return mMode;
}
