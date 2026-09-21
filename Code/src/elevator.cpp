#include "elevator.h"

#include "constants.h"

elevator::elevator(std::uint8_t port) 
: elevatorMotor(port)
, mState(manualControl)
, mSetpoint(0.0) 
{
    
}

void elevator::update(int time) {
    // Implementation for updating elevator state
}

void elevator::runManual(double input) {
    // Implementation for running elevator manually
}

void elevator::moveToSetpoint(double setpoint) {
    // Implementation for moving elevator to a setpoint
}

void elevator::resetPosition() {
    elevatorMotor.move_absolute(0, elevator_const::speed);
}

elevator::state elevator::getState() {
    return mState;
}