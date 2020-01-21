#include "CameraInfo.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/RotationMatrix.h"

void CameraInfo::updateFocalLength()
{
    focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
    focalLengthInv = 1.f / focalLength;
    focalLenPow2 = sqr(focalLength);

    focalLengthHeight = height / (2.f * std::tan(openingAngleHeight / 2.f));
    focalLengthHeightInv = 1.f / focalLengthHeight;
}