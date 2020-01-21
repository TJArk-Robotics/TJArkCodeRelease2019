#pragma once

#include "JointAngles.h"
#include "StiffnessData.h"
#include <cmath>

class JointRequest : public JointAngles
{
public:
    JointRequest();
    /** Initializes this instance with the mirrored data of other. */
    void mirror(const JointRequest &other);

    /** Returns the mirrored angle of joint. */
    Angle mirror(const Joints::Joint joint) const;

    /** Checkes if the JointRequest is valide. */
    bool isValid() const;

    StiffnessData stiffnessData;
};

class HeadJointRequest : public JointRequest
{
};

class ArmJointRequest : public JointRequest
{
};

class LegJointRequest : public JointRequest
{
public:
    LegJointRequest()
    {
        angles[0] = JointAngles::ignore;
        angles[1] = JointAngles::ignore;
    }
};

class StandArmRequest : public ArmJointRequest
{
};

class StandLegRequest : public LegJointRequest
{
};

class NonArmMotionEngineOutput : public JointRequest
{
};

inline JointRequest::JointRequest()
{
    angles.fill(off);
}

inline void JointRequest::mirror(const JointRequest &other)
{
    JointAngles::mirror(other);
    stiffnessData.mirror(other.stiffnessData);
}

inline Angle JointRequest::mirror(const Joints::Joint joint) const
{
    return JointAngles::mirror(joint);
}

inline bool JointRequest::isValid() const
{
    const char* JointName[Joints::NumOfJoint] = {
        "headYaw", "headPitch", 
        "lShoulderPitch", "lShoulderRoll", "lElbowYaw", "lElbowRoll", "lWristYaw", "lHand",
        "rShoulderPitch", "rShoulderRoll", "rElbowYaw", "rElbowRoll", "rWristYaw", "rHand",
        "lHipYawPitch", "lHipRoll", "lHipPitch", "lKneePitch", "lAnklePitch", "lAnkleRoll",
        "rHipYawPitch", "rHipRoll", "rHipPitch", "rKneePitch", "rAnklePitch", "rAnkleRoll"
    };
    bool isValid = true;
    for (unsigned i = 0; i < Joints::NumOfJoint; i++)
    {
        if (!std::isfinite(angles[i]))
        {
            std::cerr << "Joint " << JointName[i] << " is invalid" << std::endl;
            isValid = false;
        }
    }
    return stiffnessData.isValid() && isValid;
}