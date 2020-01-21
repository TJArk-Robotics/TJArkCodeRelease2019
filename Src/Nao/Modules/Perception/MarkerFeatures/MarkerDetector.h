#pragma once 

#include "Representations/Modeling/ArucoMarker.h"
#include "Tools/Module/Blackboard.h"

class MarkerDetectorBase
{
public:
    /* REQUIES */
    cv::Mat &theUsingImage = Blackboard::getInstance().usingImage;

    /* PROVIDES */
    // ArucoMarker &_theArucoMarker = Blackboard::getInstance().arucoMarker;
};

class MarkerDetector : public MarkerDetectorBase
{
public:
    void update();
    void update(ArucoMarker &arucoMarker);

    bool detect(cv::Mat &im, ArucoMarker &arucoMarker);
};