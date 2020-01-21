#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/RobotParts/Legs.h"

class WalkKick
{
public:
    class KeyFrame
    {
    public:
        KeyFrame(float phase, Vector3f position, Vector3a rotation) : phase(phase), position(position), rotation(rotation) 
        {
            onRead();
        }

        void onRead();
        float phase;
        Vector3f position;
        Vector3a rotation;
    };

    WalkKick()
    {
        kickLeg = Legs::left;
        requiresPrestep = true;
        preStepSize = Pose2f(0_deg, Vector2f(75.f, 0.f));
        duration = 0.4;
        stepSize = Pose2f(0_deg, Vector2f(50.f, 0.f));
        origin = Vector2f(20.f, 0.f);

        /** Using push_back */
        // keyFrames.push_back(KeyFrame(0.40, Vector3f(0.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));
        // keyFrames.push_back(KeyFrame(0.45, Vector3f(0.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));
        // keyFrames.push_back(KeyFrame(0.50, Vector3f(5.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));
        // keyFrames.push_back(KeyFrame(0.55, Vector3f(70.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));
        // keyFrames.push_back(KeyFrame(0.60, Vector3f(60.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));
        // keyFrames.push_back(KeyFrame(0.65, Vector3f(40.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg)));

        /** C++ new member function emplace_back */
        keyFrames.emplace_back(0.40, Vector3f(0.f, 0.f, 0.f), Vector3a(0_deg, 0_deg, 0_deg));
        keyFrames.emplace_back(0.45, Vector3f(0.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg));
        keyFrames.emplace_back(0.50, Vector3f(5.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg));
        keyFrames.emplace_back(0.55, Vector3f(70.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg));
        keyFrames.emplace_back(0.60, Vector3f(60.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg));
        keyFrames.emplace_back(0.65, Vector3f(40.f ,0.f ,10.f), Vector3a(0_deg, 0_deg, 0_deg));

        onRead();
    }

    void onRead();

    Legs::Leg kickLeg = Legs::left;
    bool requiresPrestep = false;
    Pose2f preStepSize;
    float duration = 0.f;
    Pose2f stepSize;
    Vector2f origin = Vector2f::Zero();
    std::vector<KeyFrame> keyFrames;
};

class WalkKicks
{
public:
    enum Type
    {
        none, 
        forward,
        NumOfType
    };
    std::array<WalkKick, NumOfType> kicks;
};

class WalkKickVariant
{
public:
    WalkKickVariant() = default;
    WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg);

    bool operator==(const WalkKickVariant &other) const;

    WalkKicks::Type kickType = WalkKicks::none;
    Legs::Leg kickLeg = Legs::left;
};

inline WalkKickVariant::WalkKickVariant(WalkKicks::Type kickType, Legs::Leg kickLeg) : kickType(kickType), kickLeg(kickLeg) {}

inline bool WalkKickVariant::operator==(const WalkKickVariant &other) const
{
    return kickType == other.kickType && kickLeg == other.kickLeg;
}

inline void WalkKick::KeyFrame::onRead()
{
    ASSERT(phase > 0.f && phase < 1.f);
}

inline void WalkKick::onRead()
{
    if (keyFrames.size() > 0)
    {
        ASSERT(duration > 0.f);
        auto iter = keyFrames.begin();
        auto nextIter = iter + 1;
        auto end = keyFrames.end();
        while(nextIter < end)
        {
            ASSERT(iter->phase < nextIter->phase);
            nextIter = ++iter + 1;
        }
    }
}