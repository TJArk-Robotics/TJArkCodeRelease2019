#pragma once

#include "Tools/RobotParts/Joints.h"
#include <array>
#include <iostream>

class StiffnessData
{
public:
    enum
    {
        useDefault = -1
    };

    /** The constructor resets all data to its default value. */
    StiffnessData();

    /**
   * The method returns the stiffness of the mirror (left/right) of the given joint.
   * @param joint The joint the mirror of which is returned.
   * @return The output stiffness of the mirrored joint.
   */
    int mirror(const Joints::Joint joint) const;

    /** Initializes this instance with the mirrored values of other  */
    void mirror(const StiffnessData &other);

    /** This function resets the stiffness for all joints to the default value. */
    void resetToDefault();

    /** Checks wheather all stiffnesses are in the rangel [0, 100] or have the value useDefault. */
    bool isValid() const;

    std::array<int, Joints::NumOfJoint> stiffnesses; /**< The custom stiffnesses for each joint (in %). Range: [0, 100]. */
};

class StiffnessSettings : public StiffnessData
{
};

inline StiffnessData::StiffnessData()
{
    resetToDefault();
    stiffnesses[Joints::headYaw] = 70;
    stiffnesses[Joints::headPitch] = 70;

    stiffnesses[Joints::lShoulderPitch] = 10;
    stiffnesses[Joints::lShoulderRoll] = 10;
    stiffnesses[Joints::lElbowRoll] = 10;
    stiffnesses[Joints::lElbowYaw] = 10;
    stiffnesses[Joints::lWristYaw] = 60;
    stiffnesses[Joints::lHand] = 40;
    
    stiffnesses[Joints::rShoulderPitch] = 10;
    stiffnesses[Joints::rShoulderRoll] = 10;
    stiffnesses[Joints::rElbowYaw] = 60;
    stiffnesses[Joints::rElbowRoll] = 10;
    stiffnesses[Joints::rWristYaw] = 60;
    stiffnesses[Joints::rHand] = 40;
    
    stiffnesses[Joints::lHipYawPitch] = 75;
    stiffnesses[Joints::lHipRoll] = 75;
    stiffnesses[Joints::lHipPitch] = 75;
    stiffnesses[Joints::lKneePitch] = 75;
    stiffnesses[Joints::lAnklePitch] = 75;
    stiffnesses[Joints::lAnkleRoll] = 75;
    
    stiffnesses[Joints::rHipYawPitch] = 75;
    stiffnesses[Joints::rHipRoll] = 75;
    stiffnesses[Joints::rHipPitch] = 75;
    stiffnesses[Joints::rKneePitch] = 75;
    stiffnesses[Joints::rAnklePitch] = 75;
    stiffnesses[Joints::rAnkleRoll] = 75;
}

inline int StiffnessData::mirror(const Joints::Joint joint) const
{
    switch (joint)
    {
    case Joints::lShoulderPitch:
    case Joints::lShoulderRoll:
    case Joints::lElbowYaw:
    case Joints::lElbowRoll:
    case Joints::lWristYaw:
    case Joints::lHand:
        return stiffnesses[joint - Joints::lShoulderPitch + Joints::rShoulderPitch];
    case Joints::rShoulderPitch:
    case Joints::rShoulderRoll:
    case Joints::rElbowYaw:
    case Joints::rElbowRoll:
    case Joints::rWristYaw:
    case Joints::rHand:
        return stiffnesses[joint - Joints::rShoulderPitch + Joints::lShoulderPitch];
    case Joints::lHipYawPitch:
    case Joints::lHipRoll:
    case Joints::lHipPitch:
    case Joints::lKneePitch:
    case Joints::lAnklePitch:
    case Joints::lAnkleRoll:
        return stiffnesses[joint - Joints::lHipYawPitch + Joints::rHipYawPitch];
    case Joints::rHipYawPitch:
    case Joints::rHipRoll:
    case Joints::rHipPitch:
    case Joints::rKneePitch:
    case Joints::rAnklePitch:
    case Joints::rAnkleRoll:
        return stiffnesses[joint - Joints::rHipYawPitch + Joints::lHipYawPitch];
    default:
        return stiffnesses[joint];
    }
}

inline void StiffnessData::mirror(const StiffnessData &other)
{
    for (int i = 0; i < Joints::NumOfJoint; i++)
    {
        stiffnesses[i] = other.mirror(static_cast<Joints::Joint>(i));
    }
}

inline void StiffnessData::resetToDefault()
{
    stiffnesses.fill(useDefault);
}

inline bool StiffnessData::isValid() const
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
        if (stiffnesses[i] > 100 || (stiffnesses[i] < 0 && stiffnesses[i] != useDefault))
        {
            std::cerr << "Stiffness from Joint " << JointName[i] << " is invalid" << std::endl;
            isValid = false;
        }
    }
    return isValid;
}