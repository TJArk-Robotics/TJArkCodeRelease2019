/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/RobotDimensions.h"

#include <array>

namespace ForwardKinematic
{
void calculateArmChain(Arms::Arm arm, const JointAngles &joints, const RobotDimensions &robotDimensions, std::array<Pose3f, Limbs::NumOfLimb> &limbs);
void calculateLegChain(Legs::Leg leg, const JointAngles &joints, const RobotDimensions &robotDimensions, std::array<Pose3f, Limbs::NumOfLimb> &limbs);
void calculateHeadChain(const JointAngles &joints, const RobotDimensions &robotDimensions, std::array<Pose3f, Limbs::NumOfLimb> &limbs);
}; // namespace ForwardKinematic
