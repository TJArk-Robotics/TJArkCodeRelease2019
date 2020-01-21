#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Module/Blackboard.h"

class RobotCameraMatrixProviderBase
{
public:
    /* REQUIRES */
    const CameraCalibration &theCameraCalibration = Blackboard::getInstance().cameraCalibration;
    const CameraInfo &theCameraInfo = Blackboard::getInstance().cameraInfo;
    const JointAngles &theJointAngles = Blackboard::getInstance().jointAngles;
    const RobotDimensions &theRobotDimensions = Blackboard::getInstance().robotDimensions;

    /* PROVIDES */
    // RobotCameraMatrix &_theRobotCameraMatrix = Blackboard::getInstance().robotCameraMatrix;
};

class RobotCameraMatrixProvider : public RobotCameraMatrixProviderBase
{
public:
    void update();
    void update(RobotCameraMatrix &robotCameraMatrix);
};