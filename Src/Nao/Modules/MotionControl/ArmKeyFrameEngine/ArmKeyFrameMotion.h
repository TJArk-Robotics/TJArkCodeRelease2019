/**
 * Class to represent an Arm Motion. It consists of an array of
 * angle+stiffness settings to command consecutively to an arm. This class
 * is used by the ArmKeyFrameEngine to read definitions of arm motions from
 * a configuration file.
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Angle.h"
#include "Tools/Module/Blackboard.h"

#include <vector>

class ArmKeyFrameMotion
{
public:
    class ArmAngles
    {
    public:
        ArmAngles()
        {
            // angles
            angles.push_back(0.f);
            angles.push_back(0.f);
            angles.push_back(0.f);
            angles.push_back(0.f);
            angles.push_back(0.f);
            angles.push_back(0.f);

            // stiffness
            stiffness.push_back(0);
            stiffness.push_back(0);
            stiffness.push_back(0);
            stiffness.push_back(0);
            stiffness.push_back(0);
            stiffness.push_back(0);
        }

        ArmAngles(std::vector<Angle> _angles, std::vector<int> _stiffness)
        {
            angles = _angles;
            stiffness = _stiffness;
        }

        std::vector<Angle> angles;  /**< Array of size 6, containing target angles for shoulder+elbow joint */
        std::vector<int> stiffness; /**< Array of size 6, containing stiffness data to set while targetting the above angles */
        int steps = 4;              /**< Duration in motion frame for reaching the target angles from the current position */
    };

    ArmKeyFrameMotion reverse(ArmAngles defaultPos)
    {
        ASSERT(id != ArmKeyFrameRequest::reverse);

        ArmKeyFrameMotion result;
        result.id = ArmKeyFrameRequest::reverse;

        if (id == ArmKeyFrameRequest::useDefault)
        {
            result.states = std::vector<ArmAngles>(states);
            return result;
        }

        result.states = std::vector<ArmAngles>();

        // add states in reverse order and skip the last state
        for (std::vector<ArmAngles>::reverse_iterator it = ++states.rbegin(); it != states.rend(); ++it)
        {
            result.states.push_back(*it);
        }
        // default position is the last state
        result.states.push_back(defaultPos);
        return result;
    }

    ArmAngles &getTargetState()
    {
        return *states.rbegin();
    }

    ArmKeyFrameRequest::ArmKeyFrameID id; /** Unique id of this motion. */
    std::vector<ArmAngles> states;        /** Array of states to move the arm to */
};