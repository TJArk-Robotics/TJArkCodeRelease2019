#pragma once

#include "Provider.h"
#include "Tools/Module/Blackboard.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
#include "Tools/Module/ModuleManager.h"

#include <iostream>

class Adapter
{
protected:
    Blackboard* blackboard;
    Blackboard bb;

    /** Provider */
    NaoProvider naoProvider;
    ModuleManager moduleManager;
    
    Adapter(Blackboard* bb) : blackboard(bb) 
    {
        Blackboard::setInstance(&this->bb);
    }
    virtual ~Adapter() { }
public:
    virtual void tick() {}
};