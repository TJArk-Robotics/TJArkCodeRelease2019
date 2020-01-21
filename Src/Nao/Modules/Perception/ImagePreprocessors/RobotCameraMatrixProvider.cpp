#include "RobotCameraMatrixProvider.h"

// Provider
#include "Modules/Infrastructure/JointAnglesProvider/JointAnglesProvider.h"

void RobotCameraMatrixProvider::update()
{
    // CameraCalibration

    // CameraInfo

    // JointAngles
    if (!Blackboard::getInstance().updatedMap["JointAngles"])
    {
        JointAngles &_theJointAngles = Blackboard::getInstance().jointAngles;
        JointAnglesProvider jointAnglesProvider;
        jointAnglesProvider.update(_theJointAngles);
        Blackboard::getInstance().updatedMap["JointAngles"] = true;
    }
    // RobotDimensions
}

void RobotCameraMatrixProvider::update(RobotCameraMatrix &robotCameraMatrix)
{
    update();
    robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theJointAngles.angles[Joints::headYaw], theJointAngles.angles[Joints::headPitch], theCameraCalibration, theCameraInfo.camera == CameraInfo::upper);
}