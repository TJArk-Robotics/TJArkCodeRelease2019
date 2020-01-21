/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"

class DynPoint
{
public:
    DynPoint() = default;
    DynPoint(int limb, int phaseNumber, int duration, const Vector3f &translation, const Vector3f &angle, const Vector3f &odometryOffset);
    DynPoint(int limb, int phaseNumber, const Vector3f &translation, const int duration = -1);

    bool operator==(const DynPoint &other) const;

    int limb;
    int phaseNumber;
    int duration;
    Vector3f translation = Vector3f::Zero();
    Vector3f angle = Vector3f::Zero();
    Vector3f odometryOffset = Vector3f::Zero();
};

inline bool DynPoint::operator==(const DynPoint &other) const
{
    return limb == other.limb &&
           phaseNumber == other.phaseNumber &&
           duration == other.duration &&
           translation == other.translation &&
           angle == other.angle &&
           odometryOffset == other.odometryOffset;
}

inline DynPoint::DynPoint(int limb, int phaseNumber, int duration, const Vector3f &translation,
                          const Vector3f &angle, const Vector3f &odometryOffset) : limb(limb), phaseNumber(phaseNumber), duration(duration),
                                                                                   translation(translation), angle(angle), odometryOffset(odometryOffset)
{
}

inline DynPoint::DynPoint(int limb, int phaseNumber, const Vector3f &translation, const int duration) : limb(limb), phaseNumber(phaseNumber), duration(duration), translation(translation)
{
}

class KickRequest
{
public:
    enum KickMotionID
    {
        kickForward,
        newKick,
        none,

        NumOfKickMotionID
    };

    static KickMotionID getKickMotionFromName(const char *name);

    KickMotionID kickMotionTYpe = none;
    bool mirror = false;
    bool armsBackFix = false;
    bool autoProceed = false;
    bool boost = false;

    std::vector<DynPoint> dynPoints;
};

class Continuation
{
public:
    KickRequest::KickMotionID kickType = KickRequest::none;
    bool mirror = false;
};

using stdVectorContinuation = std::vector<Continuation>;