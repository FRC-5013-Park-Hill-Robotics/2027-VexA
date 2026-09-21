#include "api.h"

#include "piston.h"

#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

class endEffector{
public:
    endEffector(std::uint8_t wristPort, std::uint8_t clawPort);
    void toggleWrist();
    void toggleClaw();
    void activateWrist(bool state);
    void activateClaw(bool state);
private:
    piston mWrist;
    piston mClaw;
};

#endif