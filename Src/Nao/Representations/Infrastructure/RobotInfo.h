/**
 * @file RobotInfo.h
 * The file declares a struct that encapsulates the structure RobotInfo defined in
 * the file RoboCupGameControlData.h that is provided with the GameController.
 * It also maps the robot's name on the robot's model.
 *
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 * @author <a href="mailto:aschreck@informatik.uni-bremen.de">André Schreck</a>
 */

#pragma once

#include "Tools/Settings.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

class RobotInfo : public RoboCup::RobotInfo
{
public:
    enum NaoVersion
    {
        V32,
        V33,
        V4,
        V5,
        V60,
        NumOfNaoVersion
    };

    enum NaoType
    {
        H21,
        H25,
        NumOfNaoType
    };

    enum RobotFeature
    {
        hands,
        grippyFingers,
        wristYaws,
        zAngle,
        zGyro,
        tactileHandSensors,
        tactileHeadSensors,
        headLEDs,
        NumOfRobotFeature
    };

    int number; /**< The number of the robot */

    NaoVersion headVersion = V60;
    NaoType headType = H25;
    NaoVersion bodyVersion = V60;
    NaoType bodyType = H25;

    RobotInfo();

    bool hasFeature(const RobotFeature feature) const;
};