#pragma once

// All Provider
#include "Modules/BehaviorControl/LEDHandler/LEDHandler.h"
#include "Modules/BehaviorControl/CameraControlEngine/CameraControlEngine.h"

#include "Modules/Infrastructure/JointAnglesProvider/JointAnglesProvider.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"

#include "Modules/Modeling/SelfLocator/SelfLocator.h"

#include "Modules/MotionControl/ArmKeyFrameEngine/ArmKeyFrameEngine.h"
#include "Modules/MotionControl/MotionCombinator/ArmMotionCombinator.h"
#include "Modules/MotionControl/MotionCombinator/HeadMotionCombinator.h"
#include "Modules/MotionControl/MotionCombinator/LegMotionCombinator.h"
#include "Modules/MotionControl/MotionCombinator/MotionCombinator.h"

#include "Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.h"
#include "Modules/MotionControl/MotionSelector/MotionSelector.h"
#include "Modules/MotionControl/SpecialActions/SpecialActions.h"
#include "Modules/MotionControl/WalkingEngine/Walk2014Generator.h"
#include "Modules/MotionControl/WalkingEngine/WalkingEngine.h"

#include "Modules/Perception/MarkerFeatures/MarkerDetector.h"

#include "Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
#include "Modules/Sensing/FootSupportProvider/FootSupportProvider.h"
#include "Modules/Sensing/GroundContactDetector/GroundContactDetector.h"
#include "Modules/Sensing/InertialDataProvier/InertialDataProvider.h"
#include "Modules/Sensing/RobotModelProvider/RobotModelProvider.h"

class ModuleManager
{
public:

    ModuleManager() { theInstance = this; }

    static thread_local ModuleManager *theInstance;

    ArmKeyFrameEngine armKeyFrameEngine;
    ArmMotionCombinator armMotionCombinator;

    CameraControlEngine cameraControlEngine;
    
    FallDownStateProvider fallDownStateProvider;
    FootSupportProvider footSupportProvider;
    
    GroundContactDetector groundContactDetector;
    
    HeadMotionCombinator headMotionCombinator;
    HeadMotionEngine headMotionEngine;
    
    InertialDataProvider inertialDataProvider;
    
    JointAnglesProvider jointAnglesProvider;
    
    LEDHandler ledHandler;
    LegMotionCombinator legMotionCombinator;
    
    MarkerDetector markerDetector;
    MotionCombinator motionCombinator;
    MotionSelector motionSelector;
    
    Walk2014Generator walk2014Generator;
    WalkingEngine walkingEngine;
    
    RobotModelProvider robotModelProvider; 
    
    SelfLocator selfLocator;
    SpecialActions specialActions;
};