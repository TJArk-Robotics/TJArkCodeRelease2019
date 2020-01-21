#pragma once

#include "Tools/Math/Eigen.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Blackboard.h"

class CameraControlEngineBase
{
public:
    /* REQUIRES */
    const RobotDimensions &theRobotDimensions = Blackboard::getInstance().robotDimensions;
    const CameraCalibration &theCameraCalibration = Blackboard::getInstance().cameraCalibration;
    const HeadLimits &theHeadLimits = Blackboard::getInstance().headLimits;
    const HeadMotionRequest &theHeadMotionRequest = Blackboard::getInstance().headMotionRequest;
    const RobotCameraMatrix &theRobotCameraMatrix = Blackboard::getInstance().robotCameraMatrix;
    const RobotModel &theRobotModel = Blackboard::getInstance().robotModel;
    const TorsoMatrix &theTorsoMatrix = Blackboard::getInstance().torsoMatrix;

    /* PROVIDES */
    // HeadAngleRequest &_theHeadAngleRequest = Blackboard::getInstance().headAngleRequest;

    Angle moveHeadThreshold = 0.18;
    Angle defaultTilt = 0.38;
};

class CameraControlEngine : public CameraControlEngineBase
{
public:
    CameraControlEngine();

    Rangea panBounds;

    void update();
    void update(HeadAngleRequest &headAngleRequest);

    void calculatePanTiltAngles(const Vector3f &hip2Target, bool lowerCamera, Vector2a &panTilt) const;
    void adjustTiltBoundToShoulder(Angle pan, bool lowerCamera, Rangea &tiltBound) const;
};