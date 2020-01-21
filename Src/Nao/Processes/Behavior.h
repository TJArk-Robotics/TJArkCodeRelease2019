#pragma once

#include "Modules/BehaviorControl/BehaviorControl/Soccer.h"
#include "Modules/Infrastructure/Adapter.h"
#include "Modules/BehaviorControl/BehaviorControl/HeadControl2018/HeadControl2018.h"
#include "Representations/Infrastructure/FrameInfo.h"

class Behavior : Adapter
{
public:
    Behavior(Blackboard *bb) : Adapter(bb) 
    {
        BH_TRACE_INIT("Behavior");
    }
    Soccer soccer;
    HeadControl2018 headControl2018;
    
    void tick();
    void update();
    void send();
    void receive();
};