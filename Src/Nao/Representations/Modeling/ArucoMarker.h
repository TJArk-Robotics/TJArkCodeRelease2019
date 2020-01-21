#pragma once

#include "Tools/Math/Pose3f.h"
#include <array>

class ArucoMarker
{
public:
    enum MarkerIDs
    {
        id21,
        id253,
        others,
        NumOfMarkerIDs
    };

    class MarkerInfo
    {
    public:
        int id;
        Pose3f positionInCameraCoord;
    };

    bool isValid = false;                       // If Marker is detected or not.
    std::array<MarkerInfo, NumOfMarkerIDs> markers; // All fixed marker info
};