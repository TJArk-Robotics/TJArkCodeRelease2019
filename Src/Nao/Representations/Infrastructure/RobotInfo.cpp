#include "RobotInfo.h"
#include "Platform/Nao/NaoBody.h"
#include "Platform/BHAssert.h"

#include <cstring>

RobotInfo::RobotInfo()
{
#ifdef TARGET_ROBOT
    NaoBody naoBody;
    headVersion = naoBody.getHeadVersion();
    bodyVersion = naoBody.getBodyVersion();
    headType = naoBody.getHeadType();
    bodyType = naoBody.getBodyType();
#endif
}

bool RobotInfo::hasFeature(const RobotFeature feature) const
{
    switch (feature)
    {
    case hands:
    case wristYaws:
    case tactileHandSensors:
        return bodyType >= H25;
    case tactileHeadSensors:
    case headLEDs:
        return headType >= H25;
    case grippyFingers:
        return bodyType >= H25 && bodyVersion >= V5;
    case zAngle:
        return bodyVersion >= V5;
    case zGyro:
        return bodyVersion >= V5;
    default:
    {
        FAIL("Unknown feature.");
        return false;
    }
    }
}