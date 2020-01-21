#pragma once

#include "Tools/Module/Blackboard.h"

#include <iostream>

class Provider
{
protected:
    Provider(Blackboard *blackboard)
    {
        this->blackboard = blackboard;
    }
    Blackboard *blackboard;
    virtual ~Provider() { }
};