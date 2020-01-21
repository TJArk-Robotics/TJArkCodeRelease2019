#include "FallDownStateProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Math/Constants.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Math/Random.h"
#include "Tools/RobotParts/FsrSensors.h"
#include "Tools/RobotParts/FootShape.h"
// Provider
// #include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
// #include "Modules/Sensing/GroundContactDetector/GroundContactDetector.h"
// #include "Modules/Sensing/InertialDataProvier/InertialDataProvider.h"
// #include "Modules/Sensing/RobotModelProvider/RobotModelProvider.h"
#include "Tools/Module/ModuleManager.h"

FallDownStateProvider::FallDownStateProvider() : lastTimeSoundPlayed(theFrameInfo.time)
{
    theSensorData = useInertiaData ? &theInertialData : &theInertialSensorData;
    getSupportPolygon();
    lastTiltingEdge = tiltingEdge = getTiltingEdge();
    initUKF(measure());
}

void FallDownStateProvider::update()
{
    // update FrameInfo
    {
        FrameInfo &_theFrameInfo = Blackboard::getInstance().frameInfo;
        NaoProvider::theInstance->update(_theFrameInfo);
    }

    // Update FsrSensorData
    if (!Blackboard::getInstance().updatedMap["FsrSensorData"])
    {
        FsrSensorData &_theFsrSensorData = Blackboard::getInstance().fsrSensorData;
        NaoProvider::theInstance->update(_theFsrSensorData);
        Blackboard::getInstance().updatedMap["FsrSensorData"] = true;
    }
    // Update GroundContactState
    if (!Blackboard::getInstance().updatedMap["GroundContactState"])
    {
        GroundContactState &_theGroundContactState = Blackboard::getInstance().groundContactState;
        // GroundContactDetector groundContactDetector;
        ModuleManager::theInstance->groundContactDetector.update(_theGroundContactState);
        Blackboard::getInstance().updatedMap["GroundContactState"] = true;
    }
    // Update InertialData
    if (!Blackboard::getInstance().updatedMap["InertialData"])
    {
        InertialData &_theInertialData = Blackboard::getInstance().inertialData;
        // InertialDataProvider inertialDataProvider;
        ModuleManager::theInstance->inertialDataProvider.update(_theInertialData);
        Blackboard::getInstance().updatedMap["InertialData"] = true;
    }
    // Update InertialSensorData
    if (!Blackboard::getInstance().updatedMap["InertialSensorData"])
    {
        InertialSensorData &_theInertialSensorData = Blackboard::getInstance().inertialSensorData;
        NaoProvider::theInstance->update(_theInertialSensorData);
        Blackboard::getInstance().updatedMap["InertialSensorData"] = true;
    }
    // Update MassCalibration
    // Update RobotModel
    if (!Blackboard::getInstance().updatedMap["RobotModel"])
    {
        // RobotModelProvider robotModelProvider;
        RobotModel &_theRobotModel = Blackboard::getInstance().robotModel;
        ModuleManager::theInstance->robotModelProvider.update(_theRobotModel);
        Blackboard::getInstance().updatedMap["RobotModel"] = true;
    }
}

void FallDownStateProvider::update(FallDownState &fallDownState)
{
    update();
    dynamicNoise = (Vector5f() << positionProcessDeviation.cwiseAbs2(), velocityProcessDeviation.cwiseAbs2().cast<float>()).finished() * Constants::motionCycleTime;
    measurementNoise = (Vector5f() << positionMeasurementDeviation.cwiseAbs2(), velocityMeasurementDeviation.cwiseAbs2().cast<float>()).finished();
    theSensorData = useInertiaData ? &theInertialData : &theInertialSensorData;
    theFallDownState = &fallDownState;

    getSupportPolygon();
    tiltingEdge = getTiltingEdge();

    // transform support foot and centroid to the tilting edge coordinate system
    const Pose3f torsoToOrigin = tiltingEdge.inverse();
    for (Vector3f &point : supportPolygon)
        point = torsoToOrigin * point;
    supportFootCenter = torsoToOrigin * supportFootCenter;

    // predict and measure the next state
    if (fallDownState.state == FallDownState::upright || fallDownState.state == FallDownState::staggering)
    {
        convertToNewOrigin(lastTiltingEdge, tiltingEdge, ukf.mean);
        updateUKF();
    }
    else
    {
        const Vector5f measurement = measure();
        initUKF(measurement);
    }
    const Vector3f &comInOrigin = ukf.mean.head<3>();
    torsoUpright = (comInOrigin.head<2>() - supportFootCenter.head<2>()).norm() <= 0.7f * (-supportFootCenter.head<2>()).norm();
    torsoAboveGround = comInOrigin.z();
    falling = isFalling();

    stable = std::abs(theSensorData->gyro.x()) <= maxGyroToRegainStableState && std::abs(theSensorData->gyro.y()) <= maxGyroToRegainStableState;
    toUpright = torsoUpright && torsoAboveGround >= maxTorsoHeightToKeepSquatting && theGroundContactState.contact;
    toSquatting = torsoUpright && torsoAboveGround < minTorsoHeightToKeepUpright && theGroundContactState.contact;
    useTorsoOrientation = std::abs(theSensorData->angle.x()) >= minTorsoOrientationToDetermineDirection.x() || std::abs(theSensorData->angle.y()) >= minTorsoOrientationToDetermineDirection.y();
    direction = getFallDirection();
    lastTiltingEdge = tiltingEdge;

    // Execute behavior
    beginFrame(theFrameInfo.time);
    execute(OptionInfos::getOption("Root"));
    endFrame();
}

bool FallDownStateProvider::isFalling() const
{
    //if the com is within the subSupportPolygon the robot is in a stable position
    if (torsoUpright)
        return false;

    const Vector3f &com = ukf.mean.head<3>();
    Vector2f vel = (Vector2f() << ukf.mean.tail<2>().y(), -ukf.mean.tail<2>().x()).finished();
    Vector2f direction = (com.head<2>() - supportFootCenter.head<2>()).normalized();

    Vector3f comSigma(std::sqrt(ukf.cov(0, 0)) * sigmaArea, std::sqrt(ukf.cov(1, 1)) * sigmaArea, 0.01f);
    Vector3f bestCaseCom = (Vector3f() << com.head<2>() - (comSigma.x() * direction), 0.f).finished();

    // robot is falling, if the com is not in the support polygon
    if (!isPointInsidePolygon(bestCaseCom, supportPolygon) && ((direction.dot(vel) > minFallVectorScalar && direction.dot(vel.normalized()) > minNormalizedFallVectorScalar) || bestCaseCom.norm() > minComDistanceToDetetctFall))
    {
        return true;
    }
    //if the robot is not falling and not stable, predict the next positions of the com
    if (theFallDownState->state == FallDownState::upright || theFallDownState->state == FallDownState::staggering)
    {
        Vector2f v = ukf.mean.tail<2>();
        // v = v.norm() > maxVelForPrediction ? maxVelForPrediction * v.normalized() : v;
        if (v.norm() > maxVelForPrediction)
            v = maxVelForPrediction * v.normalized();
        UKF<5> ukfPredict = UKF<5>((Vector5f() << ukf.mean.head<3>(), v).finished());
        ukfPredict.cov = Matrix5f(ukf.cov);
        for (float timePast = 0.f; timePast < forwardingTime; timePast += Constants::motionCycleTime)
        {
            const Matrix3f I = calcInertiaTensor(tiltingEdge);
            auto dynamic = [&](Vector5f &state) {
                dynamicModel(tiltingEdge, I, state);
            };
            ukfPredict.predict(dynamic, dynamicNoise.asDiagonal());
            ukfPredict.mean.tail<2>() = velocityDiscountFactor * ukfPredict.mean.tail<2>();
            vel << ukfPredict.mean.tail<2>().y(), -ukfPredict.mean.tail<2>().x();
            direction << (ukfPredict.mean.head<2>() - supportFootCenter.head<2>()).normalized();

            if (direction.dot(vel) < minPredictFallVectorScalar || direction.dot(vel.normalized()) < minPredictNormalizedFallVectorScalar)
                return false;
            const Vector2f &futureCom = ukfPredict.mean.head<2>();
            comSigma << std::sqrt(ukfPredict.cov(0, 0)) * sigmaArea, std::sqrt(ukfPredict.cov(1, 1)) * sigmaArea, 0.01f;
            bestCaseCom << futureCom - (comSigma.x() * direction), 0.f;
            if (!isPointInsidePolygon(bestCaseCom, supportPolygon))
            {
                return true;
            }
        }
    }
    return false;
}

FallDownState::Direction FallDownStateProvider::getFallDirection() const
{
    Vector2f direction = (ukf.mean.head<2>() - supportFootCenter.head<2>());
    Angle fallDirection = std::atan2(direction.x(), direction.y());

    if (-pi_4 <= fallDirection && fallDirection <= pi_4)
        return FallDownState::left;
    if (-pi3_4 >= fallDirection || fallDirection >= pi3_4)
        return FallDownState::right;
    if (pi_4 < fallDirection && fallDirection < pi3_4)
        return FallDownState::front;
    return FallDownState::back;
}

void FallDownStateProvider::initUKF(const Vector5f &measurement)
{
    const Vector5f initMean = measurement;
    const Matrix5f noise = Matrix5f::Identity();
    ukf.init(initMean, noise);
}

void FallDownStateProvider::updateUKF()
{
    const Matrix3f I = calcInertiaTensor(tiltingEdge);
    auto dynamic = [&](Vector5f &state) {
        dynamicModel(tiltingEdge, I, state);
    };
    auto measurementModel = [](const Vector5f &state) {
        return state;
    };
    const Vector5f measurement = measure();
    ukf.predict(dynamic, dynamicNoise.asDiagonal());
    ukf.update<5>(measurement, measurementModel, measurementNoise.asDiagonal());
}

Vector5f FallDownStateProvider::measure() const
{
    const RotationMatrix torsoRotation(Rotation::Euler::fromAngles(theSensorData->angle.x(), theSensorData->angle.y(), 0.f));
    const Vector3f com = tiltingEdge.inverse() * theRobotModel.centerOfMass;
    const Vector2f vel = (torsoRotation * theSensorData->gyro.cast<float>()).head<2>();
    return (Vector5f() << com, vel).finished();
}

void FallDownStateProvider::dynamicModel(const Pose3f &originToTorso, const Matrix3f &I, Vector5f &state, float dt) const
{
    const Vector3f weightForce = (Vector3f() << 0.f, 0.f, -theMassCalibration.totalMass * Constants::g).finished();
    const Vector3f standWidth = (Vector3f() << state.head<2>(), 0.f).finished();
    const Vector3f stabilityMoment = standWidth.cross(weightForce);
    const Vector3f acc = I.inverse() * stabilityMoment;

    const Vector2f velocity = state.tail<2>() + acc.head<2>() * dt;
    const Vector2f angleOffset = velocity * dt;
    const RotationMatrix rotation(Rotation::Euler::fromAngles(angleOffset.x(), angleOffset.y(), 0.f));
    const Vector3f com = rotation * state.head<3>();
    state << com, velocity;
}

void FallDownStateProvider::convertToNewOrigin(const Pose3f &originToTorso, const Pose3f &newOriginToTorso, Vector5f &state) const
{
    const Vector3f &comToOrigin = state.head<3>();
    const Pose3f originToNewOrigin = newOriginToTorso.inverse() * originToTorso;
    state.head<3>() = originToNewOrigin.translation + comToOrigin;
}

void FallDownStateProvider::getSupportPolygon()
{
    supportPolygon.clear();
    supportFoot = theFsrSensorData.totals[Legs::left] > theFsrSensorData.totals[Legs::right] ? Legs::left : Legs::right;
    const Pose3f &leftSoleToTorso = theRobotModel.soleLeft;
    const Pose3f &rightSoleToTorso = theRobotModel.soleRight;
    const Pose3f &supportedFootSoleToTorso = supportFoot == Legs::left ? leftSoleToTorso : rightSoleToTorso;
    const Pose3f &otherSupportedFootSoleToTorso = supportFoot == Legs::left ? rightSoleToTorso : leftSoleToTorso;
    const Vector3f com = tiltingEdge.translation + ukf.mean.head<3>();

    const bool useSupportFootOnly = std::abs(com.y()) > std::abs(supportedFootSoleToTorso.translation.y()) - minComDistanceToFootCenter && sgn(com.y()) == sgn(supportedFootSoleToTorso.translation.y()) && std::abs(theFsrSensorData.totals[Legs::left] - theFsrSensorData.totals[Legs::right]) > 0.5f && !((supportedFootSoleToTorso.translation.x() < com.x() && supportedFootSoleToTorso.translation.x() < otherSupportedFootSoleToTorso.translation.x()) || (supportedFootSoleToTorso.translation.x() > com.x() && supportedFootSoleToTorso.translation.x() > otherSupportedFootSoleToTorso.translation.x()));

    if (useSupportFootOnly)
    {
        const float sign = supportFoot == Legs::left ? 1.f : -1.f;
        for (size_t i = 0; i < FootShape::polygon.size(); ++i)
            supportPolygon.push_back(supportedFootSoleToTorso * Vector3f(FootShape::polygon[i].x(), sign * FootShape::polygon[i].y(), 0.f));
    }
    else
    {
        for (size_t i = 0; i < FootShape::polygon.size(); ++i)
        {
            supportPolygon.push_back(leftSoleToTorso * Vector3f(FootShape::polygon[i].x(), FootShape::polygon[i].y(), 0.f));
            supportPolygon.push_back(rightSoleToTorso * Vector3f(FootShape::polygon[i].x(), -FootShape::polygon[i].y(), 0.f));
        }
    }
    supportPolygon = getConvexHull(supportPolygon);

    // calculate centroid of the convex, counter-clockwise ordered polygon
    const size_t size = supportPolygon.size();
    float area = 0.f, x = 0.f, y = 0.f;
    for (size_t i = 0, j = size - 1; i < size; j = i++)
    {
        const Vector3f &p1 = supportPolygon[i];
        const Vector3f &p2 = supportPolygon[j];
        float f = p1.x() * p2.y() - p2.x() * p1.y();
        area += f;
        x += (p1.x() + p2.x()) * f;
        y += (p1.y() + p2.y()) * f;
    }
    area *= 3.f;
    supportFootCenter << x / area, y / area, supportedFootSoleToTorso.translation.z();
}

Pose3f FallDownStateProvider::getTiltingEdge()
{
    if (std::isnan(tiltingEdge.translation.x()) || std::isnan(tiltingEdge.translation.y()) || std::isnan(tiltingEdge.translation.z()))
    {
        return supportFoot == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight;
    }
    const Vector2f direction = (tiltingEdge.translation + ukf.mean.head<3>() - supportFootCenter).head<2>();
    const Geometry::Line fallDirectionLine = Geometry::Line(Vector2f(supportFootCenter.head<2>()), direction.normalized());

    // search for the intersection with the support foot polygon and the line
    Vector3f intersection3D;
    for (size_t i = 0; i < supportPolygon.size(); ++i)
    {
        // check if the line intersect a line from the two points of the polygon
        Vector2f intersection2D;
        const Vector3f &p1 = supportPolygon[i];
        const Vector3f &p2 = supportPolygon[(i + 1) % supportPolygon.size()];
        const Vector2f &base = p1.head<2>();
        const Vector2f dir = p2.head<2>() - base;
        const Geometry::Line polygonLine(base, dir.normalized());

        if (Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, base) != Geometry::isPointLeftOfLine(fallDirectionLine.base, fallDirectionLine.base + fallDirectionLine.direction, p2.head<2>()) && Geometry::getIntersectionOfLines(fallDirectionLine, polygonLine, intersection2D) && (supportFootCenter.head<2>() - intersection2D).norm() > (supportFootCenter.head<2>() + fallDirectionLine.direction - intersection2D).norm())
        {
            float scalar = (intersection2D - base).norm() / dir.norm();
            intersection3D = p1 + scalar * (p2 - p1);
            break;
        }
    }
    const RotationMatrix torsoRotation(Rotation::Euler::fromAngles(theSensorData->angle.x(), theSensorData->angle.y(), 0.f));
    return Pose3f(torsoRotation.inverse(), intersection3D);
}

bool FallDownStateProvider::isPointInsidePolygon(const Vector3f &point, const std::vector<Vector3f> &polygon) const
{
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        if (cross(polygon[j], point, polygon[i]) > 0)
            return false;
    }
    return true;
}

float FallDownStateProvider::cross(const Vector3f &O, const Vector3f &A, const Vector3f &B) const
{
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

std::vector<Vector3f> FallDownStateProvider::getConvexHull(std::vector<Vector3f> &polygon) const
{
    int n = static_cast<int>(polygon.size()),
        k = 0;
    if (n == 1)
        return polygon;
    std::vector<Vector3f> hull(2 * n);

    // Sort points lexicographically
    std::sort(polygon.begin(), polygon.end(), [](const Vector3f &p1, const Vector3f &p2) { return p1.x() < p2.x() || (p1.x() == p2.x() && p1.y() < p2.y()); });

    // Build lower hull
    for (int i = 0; i < n; ++i)
    {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], polygon[i]) <= 0)
            k--;
        hull[k++] = polygon[i];
    }
    // Build upper hull
    for (int i = n - 2, t = k + 1; i >= 0; i--)
    {
        while (k >= t && cross(hull[k - 2], hull[k - 1], polygon[i]) <= 0)
            k--;
        hull[k++] = polygon[i];
    }
    hull.resize(k - 1);
    return hull;
}

Matrix3f FallDownStateProvider::calcInertiaTensor(const Pose3f &originToTorso) const
{
    Matrix3f I = Matrix3f::Zero();
    const Pose3f torsoToOrigin = originToTorso.inverse();
    for (int i = 0; i < Limbs::NumOfLimb; ++i)
    {
        const Pose3f &limbToTorso = theRobotModel.limbs[i];
        const Vector3f &massPointToLimb = theMassCalibration.masses[i].offset;
        const Vector3f massPointToOrigin = torsoToOrigin * limbToTorso * massPointToLimb;
        const float x = massPointToOrigin.x(), y = massPointToOrigin.y(), z = massPointToOrigin.z();
        Matrix3f partial = (Matrix3f() << y * y + z * z, -x * y, -x * z,
                            -y * x, x * x + z * z, -y * z,
                            -z * x, -z * y, x * x + y * y)
                               .finished();
        partial *= theMassCalibration.masses[i].mass;
        I += partial;
    }
    return I;
}

void FallDownStateProvider::setState(FallDownState::State state, FallDownState::Direction direction,
                                     Angle odometryRotationOffset)
{
    theFallDownState->state = state;
    theFallDownState->direction = direction;
    theFallDownState->odometryRotationOffset = odometryRotationOffset;
}

void FallDownStateProvider::setStateWithPossibleDirectionChange(FallDownState::State state)
{
    if (theFallDownState->direction == FallDownState::left || theFallDownState->direction == FallDownState::right)
    {
        if (theFallDownState->direction % 4 + 1 == direction)
            setState(state, direction, -pi_2);
        else if ((theFallDownState->direction + 2) % 4 + 1 == direction)
            setState(state, direction, pi_2);
        else
            setState(state, theFallDownState->direction);
    }
    else
        setState(state, theFallDownState->direction);
}

void FallDownStateProvider::playSound(const char *file)
{
    if (playSounds && theFrameInfo.getTimeSince(lastTimeSoundPlayed) > minTimeBetweenSound)
    {
        if (std::string(file) == "upright" && Random::bernoulli(0.2f))
            file = "abseits";
        SystemCall::playSound((std::string(file) + ".wav").c_str());
        lastTimeSoundPlayed = theFrameInfo.time;
    }
}
