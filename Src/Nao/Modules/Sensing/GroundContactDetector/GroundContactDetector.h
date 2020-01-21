#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Blackboard.h"

class GroundContactDetectorBase
{
public:
    /** REQUIRES */
    const FrameInfo &theFrameInfo = Blackboard::getInstance().frameInfo;
    const FsrSensorData &theFsrSensorData = Blackboard::getInstance().fsrSensorData;
    const InertialSensorData &theInertialSensorData = Blackboard::getInstance().inertialSensorData;

    /** PROVIDES */
    // GroundContactState &_theGroundContactState = Blackboard::getInstance().groundContactState;

    float minPressureToKeepContact = 0.4f;          /**< Minimum pressure on both feet for ground contact in kg. */
    float minPressureToRegainContact = 0.5f;        /**< Minimum pressure on both feet to regain ground contact in kg. */
    float minPressurePerFootToRegainContact = 0.1f; /**< Minimum pressure on both feet to regain ground contact in kg. */
    Angle maxGyroYToRegainContact = 20_deg;         /**< Maximum y gyro value considered as "not moving". */
    int maxTimeWithoutPressure = 100;               /**< Maxmimum time allowed without minimum pressure before losing contact in ms. */
    int minTimeWithPressure = 300;                  /**< Minimum time required with minimum pressure to regain contact in ms. */
};

class GroundContactDetector : public GroundContactDetectorBase
{
public:
    unsigned lastTimeWithPressure = 0;    /**< Last time when there was enough pressure while ground contact is still assumed (in ms). */
    unsigned lastTimeWithoutPressure = 0; /**< Last time when there wasn't enough pressure while ground contact is not yet assumed (in ms). */

    void update(GroundContactState &groundContactState);
    void update();
};