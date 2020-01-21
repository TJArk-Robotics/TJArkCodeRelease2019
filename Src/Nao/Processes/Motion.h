#pragma once

#include "Modules/Infrastructure/Adapter.h"
#include "Platform/Thread.h"

class Motion : public Adapter 
{
public:
    Motion(Blackboard* bb) : Adapter(bb) 
    {
        Thread::name = "Motion";
        BH_TRACE_INIT("Motion");
    }
    void tick();
    void update();

    /* reset blackboard updatedMap to false to make sure all representations are not updated 
     * before update();
     */
    void resetUpdate();
    void send();
    void receive();
    void testMotion();
};