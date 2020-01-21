#pragma once

#include "Tools/Math/Pose2f.h"

class RobotPose : public Pose2f
{
public:
    const RobotPose &operator=(const Pose2f &other)
    {
        static_cast<Pose2f &>(*this) = other;
        inversePose = other.inverse();
    }

    void onRead();
    /** Verifies that the robot pose contains valid values. */
    void verify() const;
    Pose2f inverse() const;

    enum
    {
        unknownDeviation = 100000
    };
    Pose2f inversePose;

    float validity = 0.f;                          /**< The validity of the robot pose. (0 = invalid, 1 = perfect) */
    unsigned timeOfLastConsideredFieldFeature = 0; /**< Additional information about how good this pose might be */
    float deviation = unknownDeviation;            /**< The deviation of the robot pose. */
    Matrix3f covariance = Matrix3f::Identity();    /**< The covariance matrix of the estimated robot pose. */
    unsigned timestampLastJump = 0;                /**< Timestamp of last "big change" (jump) notificaion */
};

class GroundTrueRobotPose : public RobotPose
{
public:
    unsigned timestamp = 0;
};