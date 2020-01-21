/**
 * @file RobotModel.h
 *
 * Declaration of struct RobotModel
 *
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/RobotParts/Limbs.h"

#include <array>

class RobotModel
{
public:
    RobotModel() = default;

    /**
   * Constructs the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
    RobotModel(const JointAngles &jointAngles, const RobotDimensions &robotDimensions, const MassCalibration &massCalibration);

    /**
   * Recalculates the RobotModel from given joint data.
   * @param joints The joint data.
   * @param robotDimensions The dimensions of the robot.
   * @param massCalibration The mass calibration of the robot.
   */
    void setJointData(const JointAngles &jointAngles, const RobotDimensions &robotDimensions, const MassCalibration &massCalibration);

    /**
   * Re-calculate the center of mass in this model.
   * @param massCalibration The mass calibration of the robot.
   */
    void updateCenterOfMass(const MassCalibration &massCalibration);

    std::array<Pose3f, Limbs::NumOfLimb> limbs; /**< Coordinate frame of the limbs of the robot relative to the robot's origin. */
    Pose3f soleLeft;
    Pose3f soleRight;
    Vector3f centerOfMass = Vector3f::Zero(); /**< Position of the center of mass (center of gravity) relative to the robot's origin. */
};