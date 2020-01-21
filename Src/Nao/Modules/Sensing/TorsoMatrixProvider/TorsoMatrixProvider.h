#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Blackboard.h"

class TorsoMatrixProviderBase
{
public:
    /* REQUIRES */
    const GroundContactState &theGroundContactState = Blackboard::getInstance().groundContactState;
    const InertialData &theInertialData = Blackboard::getInstance().inertialData;
    const RobotDimensions &theRobotDimensions = Blackboard::getInstance().robotDimensions;
    const RobotModel &theRobotModel = Blackboard::getInstance().robotModel;

    /* USES */
    const TorsoMatrix &theTorsoMatrix = Blackboard::getInstance().torsoMatrix;

    /* PROVIDES */
    // TorsoMatrix &_theTorsoMatrix = Blackboard::getInstance().torsoMatrix;
    // OdometryData &_theTorsoMatrix = Blackboard::getInstance().torsoMatrix;
};

class TorsoMatrixProvider : public TorsoMatrixProviderBase
{
public:
    float lastLeftFootZRotation;  /**< The last z-rotation of the left foot. */
    float lastRightFootZRotation; /**< The last z-rotation of the right foot. */

    Vector3f lastFootSpan = Vector3f::Zero(); /**< The last span between both feet. */
    Pose3f lastTorsoMatrix;                   /**< The last torso matrix for calculating the odometry offset. */

    void update();
    /** Updates the TorsoMatrix representation.
     * @ param torsoMatrix The inertia matrix representation which is updated by this module.
     */
    void update(TorsoMatrix &torsoMatrix);

    /** Updates the OdometryData representation.
     * @ param odometryData The odometry data representation which is updated by this module.
     */
    void update(OdometryData &odometryData);
};