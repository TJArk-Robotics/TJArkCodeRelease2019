#include "CameraMatrix.h"
#include "Tools/Boundary.h"
#include "Tools/Math/Geometry.h"

CameraMatrix::CameraMatrix(const Pose3f &pose) : Pose3f(pose), invPos(Pose3f::inverse()), isValid(true) {}

RobotCameraMatrix::RobotCameraMatrix(const RobotDimensions &robotDimensions, float headYaw, float headPitch, const CameraCalibration &cameraCalibration, bool upperCamera)
{
    computeRobotCameraMatrix(robotDimensions, headYaw, headPitch, cameraCalibration, upperCamera);
}

void RobotCameraMatrix::computeRobotCameraMatrix(const RobotDimensions &robotDimensions, float headYaw, float headPitch, const CameraCalibration &cameraCalibration, bool upperCamera)
{
    *this = RobotCameraMatrix();

    translate(0., 0., robotDimensions.hipToNeckLength);
    rotateZ(headYaw);
    rotateY(headPitch);
    if (upperCamera)
    {
        translate(robotDimensions.xOffsetNeckToUpperCamera, 0.f, robotDimensions.zOffsetNeckToUpperCamera);
        rotateY(robotDimensions.tiltNeckToUpperCamera + cameraCalibration.upperCameraRotationCorrection.y());
        rotateX(cameraCalibration.upperCameraRotationCorrection.x());
        rotateZ(cameraCalibration.upperCameraRotationCorrection.z());
    }
    else
    {
        translate(robotDimensions.xOffsetNeckToLowerCamera, 0.f, robotDimensions.zOffsetNeckToLowerCamera);
        rotateY(robotDimensions.tiltNeckToLowerCamera + cameraCalibration.lowerCameraRotationCorrection.y());
        rotateX(cameraCalibration.lowerCameraRotationCorrection.x());
        rotateZ(cameraCalibration.lowerCameraRotationCorrection.z());
    }
}

CameraMatrix::CameraMatrix(const Pose3f &torsoMatrix, const Pose3f &robotCameraMatrix, const CameraCalibration &cameraCalibration)
{
    computeCameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
}

void CameraMatrix::computeCameraMatrix(const Pose3f &torsoMatrix, const Pose3f &robotCameraMatrix, const CameraCalibration &cameraCalibration)
{
    static_cast<Pose3f &>(*this) = torsoMatrix;
    rotateY(cameraCalibration.bodyRotationCorrection.y());
    rotateX(cameraCalibration.bodyRotationCorrection.x());
    conc(robotCameraMatrix);
    onRead();
}

void CameraMatrix::onRead()
{
    invPos = Pose3f::inverse();
}