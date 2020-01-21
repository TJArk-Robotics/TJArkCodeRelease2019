#include "HeadControl2018.h"

HeadControl2018::HeadControl2018() : Cabsl<HeadControl2018>(&activationGraph) {}

void HeadControl2018::execute()
{
    beginFrame(theFrameInfo.time);
    if (!theGroundContactState.contact)
    {
        LookForward();
    }
    else
    {
        switch (theHeadControlMode)
        {
        case HeadControl::off:
            SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);
            break;
        case HeadControl::lookForward:
            LookForward();
            break;
        case HeadControl::lookLeftAndRight:
            LookLeftAndRight();
        default:
            break;
        }
    }
    endFrame();
}