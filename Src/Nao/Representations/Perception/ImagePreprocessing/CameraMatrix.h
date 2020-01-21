#pragma once

#include "Tools/Math/Pose3f.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Platform/SystemCall.h"

/**
 * Matrix describing transformation from center of hip to camera.
 */
class RobotCameraMatrix : public Pose3f
{
public:
    RobotCameraMatrix() = default;
    RobotCameraMatrix(const RobotDimensions &robotDimensions, float headYaw, float headPitch, const CameraCalibration &cameraCalibration, bool upperCamera);
    void computeRobotCameraMatrix(const RobotDimensions &robotDimensions, float headYaw, float headPitch, const CameraCalibration &cameraCalibration, bool upperCamera);
};

/**
 * Matrix describing transformation from ground (center between two feet) to camera.
 */
class CameraMatrix : public Pose3f
{
private:
    Pose3f invPos; /** the inverse */
public:
    CameraMatrix() = default;
    /**
     * Kind of copy-constructor.
     * @param pose The other pose.
     */
    CameraMatrix(const Pose3f &pose);
    CameraMatrix(const Pose3f &torsoMatrix, const Pose3f &robotCameraMatrix, const CameraCalibration &cameraCalibration);

    void computeCameraMatrix(const Pose3f &torsoMatrix, const Pose3f &robotCameraMatrix, const CameraCalibration &cameraCalibration);

    inline Pose3f inverse() const
    {
        return invPos;
    }

    void onRead();

    bool isValid = true; /**< Matrix is only valid if motion was stable. */
};