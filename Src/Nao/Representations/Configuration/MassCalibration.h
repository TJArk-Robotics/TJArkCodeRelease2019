/**
 * @file MassCalibration.h
 * Declaration of a struct for representing the relative positions and masses of mass points.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/Limbs.h"

#include <array>

class MassCalibration
{
public:
    MassCalibration()
    {
        masses[Limbs::neck] = MassInfo(78.42f, Vector3f(-0.01f, 0.f, -27.42f));
        masses[Limbs::head] = MassInfo(659.37f, Vector3f(1.09f, 1.46f, 57.19f));
        masses[Limbs::shoulderLeft] = MassInfo(93.04f, Vector3f(-1.65f, -26.63f, 0.14f));
        masses[Limbs::bicepsLeft] = MassInfo(157.77f, Vector3f(24.55f, 5.63f, 3.3f));
        masses[Limbs::elbowLeft] = MassInfo(64.83f, Vector3f(-27.44f, 0.f, -0.14f));
        masses[Limbs::foreArmLeft] = MassInfo(77.61, Vector3f(25.56, 2.81, 0.76));
        masses[Limbs::wristLeft] = MassInfo(185.33f, Vector3f(34.34f, -0.88f, 3.08f));
        masses[Limbs::shoulderRight] = MassInfo(93.04f, Vector3f(-1.65f, 26.63f, 0.14f));
        masses[Limbs::bicepsRight] = MassInfo(157.77f, Vector3f(24.55f, -5.63f, 3.3f));
        masses[Limbs::elbowRight] = MassInfo(64.83f, Vector3f(-27.44f, 0.f, -0.14f));
        masses[Limbs::foreArmRight] = MassInfo(77.61f, Vector3f(25.56f, -2.81f, 0.76f));
        masses[Limbs::wristRight] = MassInfo(185.33f, Vector3f(34.34f, 0.88f, 3.08f));
        masses[Limbs::pelvisLeft] = MassInfo(69.81f, Vector3f(-7.81f, -11.14f, 26.61f));
        masses[Limbs::hipLeft] = MassInfo(140.53f, Vector3f(-15.49f, 0.29f, -5.15f));
        masses[Limbs::thighLeft] = MassInfo(389.68f, Vector3f(1.38f, 2.21f, -53.73f));
        masses[Limbs::tibiaLeft] = MassInfo(301.42f, Vector3f(4.53f, 2.25f, -49.36f));
        masses[Limbs::ankleLeft] = MassInfo(134.16f, Vector3f(0.45f, 0.29f, 6.85f));
        masses[Limbs::footLeft] = MassInfo(171.84f, Vector3f(25.42f, 3.3f, -32.39f));
        masses[Limbs::pelvisRight] = MassInfo(69.81f, Vector3f(-7.81f, 11.14f, 26.61f));
        masses[Limbs::hipRight] = MassInfo(140.53f, Vector3f(-15.49f, -0.29f, -5.15f));
        masses[Limbs::thighRight] = MassInfo(389.68f, Vector3f(1.38f, -2.21f, -53.73f));
        masses[Limbs::tibiaRight] = MassInfo(301.42f, Vector3f(4.53f, -2.25f, -49.36f));
        masses[Limbs::ankleRight] = MassInfo(134.16f, Vector3f(0.45f, -0.29f, 6.85f));
        masses[Limbs::footRight] = MassInfo(171.84f, Vector3f(25.42f, -3.3f, -32.39f));
        masses[Limbs::torso] = MassInfo(1049.6f, Vector3f(-4.13f, 0.f, 128.42f));

        onRead();
    }
    class MassInfo
    {
    public:
        MassInfo() = default;
        MassInfo(float m, Vector3f o) : mass(m), offset(o) {}
        float mass = 0.f;                   /**< The mass of this limb (in g). */
        Vector3f offset = Vector3f::Zero(); /**< The offset of the center of mass of this limb relative to its hinge. */
    };
    float totalMass = 0.f; /**< The total mass of the Robot (in g). */
    void onRead();
    std::array<MassInfo, Limbs::NumOfLimb> masses;
};

inline void MassCalibration::onRead()
{
    totalMass = 0.f;
    for (int i = 0; i < Limbs::NumOfLimb; i++)
    {
        totalMass += masses[i].mass;
    }
}