#include "api.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class elevator{
public:
    enum mode{
        manual,
        automatic
    };

    elevator(std::uint8_t left_port, std::uint8_t right_port);
    void update();

    void run(int direction);
    int getRunning();
    void slow(bool pSlow);

    void goToPosition(float position);
    void zeroPosition();
    float getCurrentPosition();

    bool isAtPosition();
    mode getMode();

private:
    pros::Motor elevatorMotorLeft;
    pros::Motor elevatorMotorRight;
    int mRunning;
    bool mSlow;

    float mCurrentPosition;
    float mTargetPosition;

    mode mMode;
};

#endif