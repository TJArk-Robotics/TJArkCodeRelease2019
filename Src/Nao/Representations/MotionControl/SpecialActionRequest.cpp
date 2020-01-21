/**
 * @file Representations/MotionControl/SpecialActionRequest.cpp
 * This file implements a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#include "SpecialActionRequest.h"
#include <cstring>

SpecialActionRequest::SpecialActionID SpecialActionRequest::getSpecialActionFromName(const char* name)
{
    if (!strcmp(name, "playDead"))
    {
        return playDead;
    }
    else if (!strcmp(name, "sitDown"))
    {
        return sitDown;
    }
    else if (!strcmp(name, "stand"))
    {
        return stand;
    }
    else if (!strcmp(name, "standHigh"))
    {
        return standHigh;
    }
    else if (!strcmp(name, "standHighLookUp"))
    {
        return standHighLookUp;
    }
    else if (!strcmp(name, "getUpEngineDummy"))
    {
        return getUpEngineDummy;
    }
    else
    {
        return NumOfSpecialActionID;
    }
}
