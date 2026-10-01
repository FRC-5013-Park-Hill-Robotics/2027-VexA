#include "api.h"

#include "MiniPID.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class elevator{
public:
    enum mode{
        manual,
        automatic
    };

    elevator(std::uint8_t left_port, std::uint8_t right_port, std::uint8_t third_port);
    void update();

    void run(int direction);
    int getRunning();
    void slow(bool pSlow);

    void goToPosition(float position);
    void goToSetpoint(int setpoint_index);
    void incrementSetpoint(int setpoint_index_increment);
    void zeroPosition();

    float getCurrentPosition();
    float getTargetPosition();
    int getCurrentSetpointIndex();

    bool isAtPosition();
    mode getMode();

private:
    pros::Motor elevatorMotorLeft;
    pros::Motor elevatorMotorRight;
    pros::Motor elevatorMotorThird;

    MiniPID mPID;

    int mRunning;
    bool mSlow;

    float mCurrentPosition;
    float mTargetPosition;
    int mCurrentSetpointIndex;

    mode mMode;
};

#endif