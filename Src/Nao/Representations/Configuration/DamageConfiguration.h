#pragma once

#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/Joints.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Math/Pose2f.h"
#include "GetupMotion.h"
#include <cstring>
#include <array>

class DamageConfigurationHead
{
public:
    DamageConfigurationHead()
    {
        std::memset(audioChannelsDefect, 0, sizeof(audioChannelsDefect));
        audioChannelsDefect[0] = false;
        audioChannelsDefect[1] = false;
        audioChannelsDefect[2] = false;
        audioChannelsDefect[3] = false;
    }

    bool audioChannelsDefect[4];
};

class DamageConfigurationBody
{
public:
    DamageConfigurationBody()
    {
        noFieldGenuflect = false;
        brokenStandUp = allFine;

        getUpFront.push_back(GetUpMotions::frontFreeJoints);
        getUpFront.push_back(GetUpMotions::frontFreeJoints);
        getUpFront.push_back(GetUpMotions::frontFreeJoints);

        getUpBack.push_back(GetUpMotions::backFreeJoints);
        getUpBack.push_back(GetUpMotions::backFreeJoints);
        getUpBack.push_back(GetUpMotions::backFreeJoints);

        optionalLineVersionFront = 0;
        optionalLineVersionBack = 0;

        sides[Legs::left].weakLeg = false;
        sides[Legs::left].footBumperDefect = false;
        sides[Legs::left].armContactDefect = false;

        sides[Legs::right].weakLeg = false;
        sides[Legs::right].footBumperDefect = false;
        sides[Legs::right].armContactDefect = false;

        startTiltLeft = Vector2f(0.f, 0.f);
        startTiltRight = Vector2f(0.f, 0.f);
    }

    enum BrokenStandUp
    {
        allFine,      /**< try left and if not successful right. */
        onlyNormal,   /**< left stand foot based */
        onlyMirrored, /**< right stand foot based */
        allBroken,    /**< don't even try to stand. */
        NumOfBrokenStandUp
    };

    class Side
    {
    public:
        Side()
        {
            brokenFsrs.fill(false);
        }

        bool weakLeg = false;
        bool footBumperDefect = false;
        bool armContactDefect = false;
        std::array<bool, FsrSensors::NumOfFsrSensor> brokenFsrs;
    };

    bool noFieldGenuflect = false;
    BrokenStandUp brokenStandUp = allFine;
    GetUpMotions::GetupMotionVector getUpBack;
    GetUpMotions::GetupMotionVector getUpFront;
    float optionalLineVersionFront = 0;
    float optionalLineVersionBack = 0;
    Joints::stdVectorJoint jointsToEraseStiffness = Joints::stdVectorJoint();
    Vector2f startTiltLeft;
    Vector2f startTiltRight;
    std::array<Side, Legs::NumOfLeg> sides;
};