#pragma once

#include "Representations/Configuration/IMUCalibration.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/UnscentedKalmanFilter.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Module/Blackboard.h"

class InertialDataProviderBase
{
public:
    /* REQUIRES */
    const MotionInfo &theMotionInfo = Blackboard::getInstance().motionInfo;
    const GroundContactState &theGroundContactState = Blackboard::getInstance().groundContactState;
    const InertialSensorData &theInertialSensorData = Blackboard::getInstance().inertialSensorData;
    const IMUCalibration &theIMUCalibration = Blackboard::getInstance().imuCalibration;

    /* PROVIDE */
    // InertialData &theInertialData = Blackboard::getInstance().inertialData;

    Vector3a gyroDeviation = Vector3a::Constant(0.03_deg); // Noise of the gyro in °/s / sqrt(Hz).
    Vector3f accDeviation = Vector3f(10.f, 10.f, 10.f);
    Vector3f accDeviationWhileWalking = Vector3f(30.f, 30.f, 30.f);
    Vector3f accDeviationFactor = Vector3f(10.f, 10.f, 10.f);
    Vector3f accDynamicFilterDeviation = Vector3f(3.f, 3.f, 3.f);
    float maxMeanSquaredDeviation = 0.1f;
};

class InertialDataProvider : public InertialDataProviderBase
{
public:
    struct State : public Manifold<3>
    {
        Quaternionf orientation;

        State(const Quaternionf &orientation = Quaternionf::Identity()) : orientation(orientation)
        {
            // orientation = Quaternionf::Identity(); // Workaround, usual initialization crashes in Debug
        }

        State operator+(const Vectorf &angleAxis) const;
        State &operator+=(const Vectorf &angleAxis);
        Vectorf operator-(const State &other) const;
    };

    UKFM<State> ukf = UKFM<State>(State());

    UKF<3> filteredAccUKF = UKF<3>(Vector3f::Zero());

    Vector2a lastRawAngle = Vector2a::Zero();
    Vector3f lastAccelerometerMeasurement = Vector3f::Zero();
    bool hadAccelerometerMeasurement = false;

    RingBufferWithSum<float, 50> accelerometerLengths;
    float gravity = Constants::g_1000;

    void update();

    void update(InertialData &inertialData);

    void processAccelerometer(const Vector3f &acc);

    void processGyroscope(const Vector3a &gyro);

    void filterAcc(InertialData &id);
};