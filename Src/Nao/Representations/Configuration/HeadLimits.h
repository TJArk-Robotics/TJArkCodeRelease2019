#pragma once

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Range.h"

#include <vector>

class HeadLimits
{
public:
    bool intersectionWithShoulderPlane(const RobotCameraMatrix &robotCameraMatrix,
                                       const Vector3f &shoulderInOrigin,
                                       Angle imageTilt, Vector3f &intersection) const;

    Rangea getTiltBound(Angle pan) const;
    bool imageCenterHiddenByShoulder(const RobotCameraMatrix &robotCameraMatrix,
                                     const Vector3f &shoulderInOrigin, Angle imageTilt,
                                     float hysteresis = 0.f) const;

    bool intersectionWithShoulderEdge(const RobotCameraMatrix &robotCameraMatrix,
                                      const Vector3f &shoulderInOrigin, Vector3f &intersection) const;

    Angle maxPan() const { return intervals.back(); }
    Angle minPan() const { return intervals.front(); }

    float shoulderRadius;
    std::vector<Angle> intervals;
    std::vector<Angle> lowerBounds;
    std::vector<Angle> upperBounds;

    HeadLimits() 
    {
        shoulderRadius = 48.0f;

        // intervals
        intervals.push_back(Angle(-2.086017522f));
        intervals.push_back(Angle(-1.5269885626));
        intervals.push_back(Angle(-1.0899581179));
        intervals.push_back(Angle(-0.903033355));
        intervals.push_back(Angle(-0.756076632));
        intervals.push_back(Angle(-0.4860741967));
        intervals.push_back(Angle(0));
        intervals.push_back(Angle(0.4860741967));
        intervals.push_back(Angle(0.756076632));
        intervals.push_back(Angle(0.903033355));
        intervals.push_back(Angle(1.0899581179));
        intervals.push_back(Angle(1.5269885626));
        intervals.push_back(Angle(2.086017522));

        // lowerBounds
        lowerBounds.push_back(Angle(-0.4490732165));
        lowerBounds.push_back(Angle(-0.3300417616));
        lowerBounds.push_back(Angle(-0.4300491277));
        lowerBounds.push_back(Angle(-0.4799655443));
        lowerBounds.push_back(Angle(-0.5480333851));
        lowerBounds.push_back(Angle(-0.671951762));
        lowerBounds.push_back(Angle(-0.671951762));
        lowerBounds.push_back(Angle(-0.671951762));
        lowerBounds.push_back(Angle(-0.5480333851));
        lowerBounds.push_back(Angle(-0.4799655443));
        lowerBounds.push_back(Angle(-0.4300491277));
        lowerBounds.push_back(Angle(-0.3300417616));
        lowerBounds.push_back(Angle(-0.4490732165));

        // upperBounds
        upperBounds.push_back(Angle(0.3300417616));
        upperBounds.push_back(Angle(0.2000147323));
        upperBounds.push_back(Angle(0.3000220984));
        upperBounds.push_back(Angle(0.3300417616));
        upperBounds.push_back(Angle(0.3700098014));
        upperBounds.push_back(Angle(0.4220206131));
        upperBounds.push_back(Angle(0.5150466623));
        upperBounds.push_back(Angle(0.4220206131));
        upperBounds.push_back(Angle(0.3700098014));
        upperBounds.push_back(Angle(0.3300417616));
        upperBounds.push_back(Angle(0.3000220984));
        upperBounds.push_back(Angle(0.2000147323));
        upperBounds.push_back(Angle(0.3300417616));
    }
};