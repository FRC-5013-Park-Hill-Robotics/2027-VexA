#include "endEffector.h"

endEffector::endEffector(std::uint8_t wristPort, std::uint8_t clawPort)
: mWrist(wristPort), mClaw(clawPort)
{
}

void endEffector::toggleWrist(){
    mWrist.toggle();
}

void endEffector::toggleClaw(){
    mClaw.toggle();
}

void endEffector::activateWrist(bool state){
    mWrist.activate(state);
}

void endEffector::activateClaw(bool state){
    mClaw.activate(state);
}