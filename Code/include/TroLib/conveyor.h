#include "api.h"

#ifndef CONVEYOR_H
#define CONVEYOR_H

class conveyor{
public:
    conveyor(std::uint8_t port);
    void run(int direction);
    int getRunning();
    void slow(bool pSlow);
private:
    pros::Motor conveyorMotor;
    int running;
    bool mSlow;
};

#endif