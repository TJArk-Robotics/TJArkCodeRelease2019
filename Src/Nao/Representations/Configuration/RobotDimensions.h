/**
 * @file RobotDimensions.h
 * Description of the dimensions of the NAO robot.
 * @author Cord Niehaus
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/RobotParts/FsrSensors.h"

/**
 * This representation contains all necessary dimensions of the robot.
 * The torso coordinate frame is considert to be in the middle between the hip joints.
 */

class RobotDimensions
{
public:
    RobotDimensions()
    {
        yHipOffset = 50.f;
        upperLegLength = 100.f;
        lowerLegLength = 102.9f;
        footHeight = 45.19f;
        footLength = 110.f;
        hipToNeckLength = 211.5f;
        xOffsetNeckToLowerCamera = 50.71f;
        zOffsetNeckToLowerCamera = 17.74f;
        tiltNeckToLowerCamera = 39.7_deg;
        xOffsetNeckToUpperCamera = 58.71f;
        zOffsetNeckToUpperCamera = 63.64f;
        tiltNeckToUpperCamera = 1.2_deg;
        armOffset = Vector3f(0.f, 98.f, 185.f);
        yOffsetElbowToShoulder = 15.f;
        upperArmLength = 105.f;
        lowerArmLength = 130.f;
        xOffsetElbowToWrist = 55.95f;
        handOffset = Vector3f(57.75f, 0.f, 12.31f);
        handRadius = 32.5f;
        armRadius = 25.f;
        imuOffset = Vector3f(-8.f, 6.06f, 112.f);
        leftFsrPositions = {Vector2f(70.25f, 29.9f), Vector2f(70.25f, -23.1f), Vector2f(-30.25f, 29.9f), Vector2f(-29.65f, -19.1f)};
        rightFsrPositions = {Vector2f(70.25f, 23.1f), Vector2f(70.25f, -29.9f), Vector2f(-29.65f, 19.1f), Vector2f(-30.25f, -29.9f)};
    }
    /**
   * x-offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
    float getXOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? xOffsetNeckToLowerCamera : xOffsetNeckToUpperCamera; }

    /**
   * Height offset between the neck joint and current camera.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
    float getZOffsetNeckToCamera(bool lowerCamera) const { return lowerCamera ? zOffsetNeckToLowerCamera : zOffsetNeckToUpperCamera; }

    /**
   * Tilt of current camera against neck.
   * @param lowerCamera true, if lower camera is in use, false otherwise.
   */
    Angle getTiltNeckToCamera(bool lowerCamera) const { return lowerCamera ? tiltNeckToLowerCamera : tiltNeckToUpperCamera; }

    float yHipOffset;               //!< The y offset of the left hip.
    float upperLegLength;           //!< Length between leg joints HipPitch and KneePitch in z-direction.
    float lowerLegLength;           //!< Length between leg joints KneePitch and AnklePitch in z-direction.
    float footHeight;               //!< Height between the sole of the foot and the foot joint AnkleRoll.
    float footLength;               //!< Length from the ankle joint to the tip of the foot.
    float hipToNeckLength;          //!< Height offset between hip and joint headYaw.
    float xOffsetNeckToLowerCamera; //!< Forward offset between joint headPitch and lower camera.
    float zOffsetNeckToLowerCamera; //!< Height offset between joint headPitch and lower camera.
    Angle tiltNeckToLowerCamera;    //!< Tilt of lower camera against joint headPitch.

    float xOffsetNeckToUpperCamera; //!< Forward offset between joint headPitch and upper camera.
    float zOffsetNeckToUpperCamera; //!< Height offset between joint headPitch and upper camera.
    Angle tiltNeckToUpperCamera;    //!< Tilt of upper camera against joint headPitch.

    Vector3f armOffset;           //!< The offset of joint lShoulderPitch relative to the torso coordinate frame (y must be negated for right arm).
    float yOffsetElbowToShoulder; //!< The offset between the joints lShoulderRoll and lElbowYaw in y (must be negated for right arm).
    float upperArmLength;         //!< The length between the joints ShoulderRoll and ElbowYaw in x-direction.
    float lowerArmLength;         //!< The length of the lower arm starting at ElbowRoll.
    float xOffsetElbowToWrist;    //!< The length from Elbow to WristJoint.
    Vector3f handOffset;          //!< The offset of a hand relative to his wrist coordinate frame.
    float handRadius;             //!< The radius of a virtuel sphere a hand can span.
    float armRadius;              //!< The radius of an arm.

    Vector3f imuOffset; //!< The offset of the imu relative to the torso coordinate frame.

    std::array<Vector2f, FsrSensors::NumOfFsrSensor> leftFsrPositions;  //!< The positions of the fsr on the left foot.
    std::array<Vector2f, FsrSensors::NumOfFsrSensor> rightFsrPositions; //!< The positions of the fsr on the right foot.
};