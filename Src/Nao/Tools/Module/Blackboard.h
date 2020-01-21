/**
 * @see https://github.com/UNSWComputing/rUNSWift-2018-release/wiki/Blackboard-Serialization
 */

#pragma once

#include "Modules/BehaviorControl/BehaviorControl/Tools/HeadControlMode.h"

#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/GlobalOptions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/Infrastructure/Synchronisation.h"

#include "Representations/Modeling/ArucoMarker.h"
// #include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadAngleRequest.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

#include <array>
#include <map>
#include <opencv2/opencv.hpp>

class SynchronisationBlackboard 
{
public:
    pthread_mutex_t visionMutex;
};

class Blackboard 
{
public:
    /** http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html#StructHavingEigenMembers_othersolutions */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Blackboard();
    ~Blackboard();

    /* update map */
    // std::array<bool, NumOfRepresentationInfo> representationUpdated;
    std::map<std::string, bool> updatedMap;
    bool camTopOpen = false;
    bool camLowOpen = false;

    void initMap();
    
    /* All the representations declared here */
    ArmJointRequest armJointRequest;
    ArmKeyFrameEngineOutput armKeyFrameEngineOutput;
    ArmMotionRequest armMotionRequest;
    ArmMotionInfo armMotionInfo;
    ArmMotionSelection armMotionSelection;
    ArucoMarker arucoMarker;

    CameraInfo cameraInfo;
    CameraCalibration cameraCalibration;

    DamageConfigurationBody damageConfigurationBody;
    DamageConfigurationHead damageConfigurationHead;

    GetUpEngineOutput getUpEngineOutput;

    FallDownState fallDownState;
    FootSupport footSupport;
    FrameInfo frameInfo;
    FsrSensorData fsrSensorData;

    GlobalOptions globalOptions;
    GroundContactState groundContactState;

    HeadAngleRequest headAngleRequest;
    HeadControlMode headControlMode;
    HeadLimits headLimits;
    HeadJointRequest headJointRequest;
    HeadMotionRequest headMotionRequest;
    HeadMotionEngineOutput headMotionEngineOutput;

    InertialData inertialData;
    InertialSensorData inertialSensorData;
    IMUCalibration imuCalibration;

    JointAngles jointAngles;
    JointCalibration jointCalibration;
    JointLimits jointLimits;
    JointRequest jointRequest;
    JointSensorData jointSensorData;

    KeyStates keyStates;
    KickEngineOutput kickEngineOutput;

    LegJointRequest legJointRequest;
    LegMotionSelection legMotionSelection;
    LEDRequest ledRequest;

    MassCalibration massCalibration;
    MotionInfo motionInfo;
    MotionRequest motionRequest;

    OdometryData odometryData;
    
    RobotCameraMatrix robotCameraMatrix;
    RobotDimensions robotDimensions;
    RobotInfo robotInfo;
    RobotModel robotModel;
    RobotPose robotPose;

    SpecialActionsOutput specialActionsOutput;
    StandArmRequest standArmRequest;
    StandLegRequest standLegRequest;
    StiffnessSettings stiffnessSettings;
    SystemSensorData systemSensorData;

    TorsoMatrix torsoMatrix;

    WalkGenerator walkGenerator;
    WalkingEngineOutput walkingEngineOutput;
    WalkKicks walkKicks;

    // Test Representation
    Pose2f ballPosition;
    Synchronisation synchronisation;

    SynchronisationBlackboard locks;

    cv::Mat usingImage;
    
    cv::Mat imageTop[3];
    int newestImageTop = 0;
    int readingImageTop = 0;

    cv::Mat imageLow[3];
    int newestImageLow = 0;
    int readingImageLow = 0;

    /* Function to read a component from the Blackboard */
    template<class T> const T& read(const T *component);

    /* Write a component to the Blackboard */
    template<class T> void write(T *component, const T& value);

    static Blackboard &getInstance();
    static void setInstance(Blackboard* instance);
};