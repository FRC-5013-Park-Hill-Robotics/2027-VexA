#include "api.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class elevator{
public:
    enum state{
        manualControl,
        movingToSetpoint,
        atSetpoint
    };
public:
    elevator(std::uint8_t port);
    void update(int time);
    void runManual(double input);
    void moveToSetpoint(double setpoint);
    void resetPosition(); //Move to zero position and reset encoder
    state getState();
private:
    pros::Motor elevatorMotor;
    state mState;
    double mSetpoint;
};

#endif