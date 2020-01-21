/**
 * @file Representations/MotionControl/SpecialActionRequest.h
 * This file declares a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

class SpecialActionRequest
{
public:
    enum SpecialActionID
    {
        playDead,
        sitDown,
        stand,
        standHigh,
        standHighLookUp,
        getUpEngineDummy, // Used for debugging motion of getEngine

        NumOfSpecialActionID
    };

    static SpecialActionID getSpecialActionFromName(const char* name);

    SpecialActionID specialAction = playDead;
    bool mirror = false;
};