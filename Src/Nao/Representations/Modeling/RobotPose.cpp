#include "RobotPose.h"
#include "Platform/Time.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Projection.h"

void RobotPose::onRead()
{
    inversePose = *this;
    inversePose.invert();
}

Pose2f RobotPose::inverse() const
{
    FAIL("Use RobotPose::inversePose instead.");
    return inversePose;
}

void RobotPose::verify() const
{
    ASSERT(std::isfinite(translation.x()));
    ASSERT(std::isfinite(translation.y()));
    ASSERT(std::isfinite(rotation));
    ASSERT(rotation >= -pi);
    ASSERT(rotation <= pi);

    ASSERT(validity >= 0.f);
    ASSERT(validity <= 1.f);

    ASSERT(std::isfinite(deviation));

    ASSERT(std::isnormal(covariance(0, 0)));
    ASSERT(std::isnormal(covariance(1, 1)));
    ASSERT(std::isnormal(covariance(2, 2)));
    ASSERT(std::isfinite(covariance(0, 1)));
    ASSERT(std::isfinite(covariance(0, 2)));
    ASSERT(std::isfinite(covariance(1, 0)));
    ASSERT(std::isfinite(covariance(1, 2)));
    ASSERT(std::isfinite(covariance(2, 0)));
    ASSERT(std::isfinite(covariance(2, 1)));

    Pose2f inv = *this;
    inv.invert();
    ASSERT(inv.rotation == inversePose.rotation);
    ASSERT(inv.translation.x() == inversePose.translation.x());
    ASSERT(inv.translation.y() == inversePose.translation.y());
}
