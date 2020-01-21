#include "Blackboard.h"

static thread_local Blackboard *theInstance = nullptr;

Blackboard::Blackboard()
{
    theInstance = this;
    initMap();
}

Blackboard::~Blackboard()
{
    theInstance = nullptr;
}

template <class T>
const T &Blackboard::read(const T *component)
{
    return *component;
}

template <class T>
void Blackboard::write(T *component, const T &value)
{
    *component = value;
}

Blackboard &Blackboard::getInstance()
{
    return *theInstance;
}

void Blackboard::setInstance(Blackboard *blackboard)
{
    theInstance = blackboard;
}

void Blackboard::initMap()
{
    updatedMap.insert(std::pair<std::string, bool>("ArmJointRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("ArmMotionSelection", false));
    updatedMap.insert(std::pair<std::string, bool>("ArucoMarker", false));

    updatedMap.insert(std::pair<std::string, bool>("CameraInfo", false));
    updatedMap.insert(std::pair<std::string, bool>("CameraCalibration", false));
    updatedMap.insert(std::pair<std::string, bool>("DamageConfigurationBody", false));
    updatedMap.insert(std::pair<std::string, bool>("DamageConfigurationHead", false));

    updatedMap.insert(std::pair<std::string, bool>("GetUpEngineOutput", false));

    updatedMap.insert(std::pair<std::string, bool>("FallDownState", false));
    updatedMap.insert(std::pair<std::string, bool>("FootSupport", false));
    updatedMap.insert(std::pair<std::string, bool>("FrameInfo", false));
    updatedMap.insert(std::pair<std::string, bool>("FsrSensorData", false));

    updatedMap.insert(std::pair<std::string, bool>("GlobalOptions", false));
    updatedMap.insert(std::pair<std::string, bool>("GroundContactState", false));

    updatedMap.insert(std::pair<std::string, bool>("HeadAngleRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("HeadControlMode", false));
    updatedMap.insert(std::pair<std::string, bool>("HeadLimits", false));
    updatedMap.insert(std::pair<std::string, bool>("HeadJointRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("HeadMotionRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("HeadMotionEngineOutput", false));

    updatedMap.insert(std::pair<std::string, bool>("InertialData", false));
    updatedMap.insert(std::pair<std::string, bool>("InertialSensorData", false));
    updatedMap.insert(std::pair<std::string, bool>("IMUCalibration", false));

    updatedMap.insert(std::pair<std::string, bool>("JointAngles", false));
    updatedMap.insert(std::pair<std::string, bool>("JointCalibration", false));
    updatedMap.insert(std::pair<std::string, bool>("JointLimits", false));
    updatedMap.insert(std::pair<std::string, bool>("JointRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("JointSensorData", false));

    updatedMap.insert(std::pair<std::string, bool>("KeyStates", false));
    updatedMap.insert(std::pair<std::string, bool>("KickEngineOutput", false));

    updatedMap.insert(std::pair<std::string, bool>("LegJointRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("LegMotionSelection", false));
    updatedMap.insert(std::pair<std::string, bool>("LEDRequest", false));

    updatedMap.insert(std::pair<std::string, bool>("MassCalibration", false));
    updatedMap.insert(std::pair<std::string, bool>("MotionInfo", false));
    updatedMap.insert(std::pair<std::string, bool>("MotionRequest", false));

    updatedMap.insert(std::pair<std::string, bool>("OdometryData", false));

    updatedMap.insert(std::pair<std::string, bool>("RobotCameraMatrix", false));
    updatedMap.insert(std::pair<std::string, bool>("RobotDimensions", false));
    updatedMap.insert(std::pair<std::string, bool>("RobotInfo", false));
    updatedMap.insert(std::pair<std::string, bool>("RobotModel", false));

    updatedMap.insert(std::pair<std::string, bool>("StandArmRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("StandLegRequest", false));
    updatedMap.insert(std::pair<std::string, bool>("StiffnessSettings", false));
    updatedMap.insert(std::pair<std::string, bool>("SystemSensorData", false));

    updatedMap.insert(std::pair<std::string, bool>("TorsoMatrix", false));

    updatedMap.insert(std::pair<std::string, bool>("UsingImage", false));

    updatedMap.insert(std::pair<std::string, bool>("WalkGenerator", false));
    updatedMap.insert(std::pair<std::string, bool>("WalkingEngineOutput", false));
    updatedMap.insert(std::pair<std::string, bool>("WalkKicks", false));    
}