#pragma once

class KeyStates
{
public:
    KeyStates()
    {
        pressed.fill(false);
    }
    
    enum Key
    {
        // touch sensors:
        headFront,
        headMiddle,
        headRear,
        lHandBack,
        lHandLeft,
        lHandRight,
        rHandBack,
        rHandLeft,
        rHandRight,

        // bumpers:
        lFootLeft,
        lFootRight,
        rFootLeft,
        rFootRight,
        chest,
        NumOfKey
    };

    std::array<bool, NumOfKey> pressed;
};