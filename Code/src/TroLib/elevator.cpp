#include "elevator.h"
#include "constants.h"
#include "api.h"

using namespace TroLib;

elevator::elevator(std::uint8_t left_port, std::uint8_t right_port, std::uint8_t third_port)
: elevatorMotorLeft(left_port, elevator_const::gearset, elevator_const::units)
, elevatorMotorRight(right_port, elevator_const::gearset, elevator_const::units)
, elevatorMotorThird(third_port, elevator_const::gearset, elevator_const::units)
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
    elevatorMotorThird.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    elevatorMotorLeft.set_reversed(false);
    elevatorMotorRight.set_reversed(true);
    elevatorMotorThird.set_reversed(false);

    mPID.setOutputLimits(-1, 1); // Never more that 100% power
}

void elevator::update()
{
    mCurrentPosition = elevatorMotorLeft.get_position();
    
    if(mMode == automatic){
        mRunning = 0;
        float error = mTargetPosition - mCurrentPosition;

        if(std::abs(error) < elevator_const::position_tolerance){ 
            elevatorMotorLeft.move_velocity(0);
            elevatorMotorRight.move_velocity(0);
            elevatorMotorThird.move_velocity(0);
        } else {
            double output = mPID.getOutput(mCurrentPosition, mTargetPosition); // Assuming update is called every 20ms
            elevatorMotorLeft.move_velocity(output * elevator_const::speed);
            elevatorMotorRight.move_velocity(output * elevator_const::speed);
            elevatorMotorThird.move_velocity(output * elevator_const::speed);
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
        elevatorMotorThird.move_velocity(0);
    }
    else if(direction == 1){
        if(mSlow){
            elevatorMotorLeft.move_velocity(elevator_const::slow_speed);
            elevatorMotorRight.move_velocity(elevator_const::slow_speed);
            elevatorMotorThird.move_velocity(elevator_const::slow_speed);
        } else{
            elevatorMotorLeft.move_velocity(elevator_const::speed);
            elevatorMotorRight.move_velocity(elevator_const::speed);
            elevatorMotorThird.move_velocity(elevator_const::speed);
        }
    }
    else if(direction == -1){
        if(mSlow){
            elevatorMotorLeft.move_velocity(-elevator_const::slow_speed);
            elevatorMotorRight.move_velocity(-elevator_const::slow_speed);
            elevatorMotorThird.move_velocity(-elevator_const::slow_speed);
        } else{
            elevatorMotorLeft.move_velocity(-elevator_const::speed);
            elevatorMotorRight.move_velocity(-elevator_const::speed);
            elevatorMotorThird.move_velocity(-elevator_const::speed);
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

    position = std::clamp(
        position,
        elevator_const::min_position,
        elevator_const::max_position
    );

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
    
    //When switching from manual to auto, go to the closest setpoints rather than
    //keep previous auto setpoints
    // if(mMode == manual){
    //     if(setpoint_index_increment > 0){
    //         for(int i = std::size(elevator_const::setpoints) - 1; i >= 0; i--){
    //             //Start at highest position and itterate downwards until you find a position bellow
    //             //current. Then you know the position above it (+1) is the one above current
    //             if(mCurrentPosition < elevator_const::setpoints[i]){
    //                 newSetpointIndex = i + 1;
    //                 break;
    //             }
    //         }
    //     }
    //     if(setpoint_index_increment < 0){
    //         for(int i = 0; i < std::size(elevator_const::setpoints); i++){
    //             //Start at lowest position and itterate upwards until you find a position bellow
    //             //current. Then you know the position above it (-1) is the one bellow current
    //             if(mCurrentPosition > elevator_const::setpoints[i]){
    //                 newSetpointIndex = i - 1;
    //                 break;
    //             }
    //         }
    //     }
    // }

    if(newSetpointIndex >= 0 && newSetpointIndex < std::size(elevator_const::setpoints)){
        mMode = automatic;
        mTargetPosition = elevator_const::setpoints[newSetpointIndex];
        mCurrentSetpointIndex = newSetpointIndex;
    }
}

void elevator::zeroPosition()
{
    elevatorMotorLeft.tare_position();
    elevatorMotorRight.tare_position();

    mCurrentPosition = 0;
    mTargetPosition = 0;
    mCurrentSetpointIndex = 0;
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
