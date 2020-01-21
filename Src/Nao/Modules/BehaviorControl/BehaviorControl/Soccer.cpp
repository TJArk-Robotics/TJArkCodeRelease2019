#include "Soccer.h"
#include "Modules/Infrastructure/NaoProvider/NaoProvider.h"
#include "Platform/Thread.h"

#include <iostream>
// #include "Tools/BehaviorOptionRegistry.h"

// Soccer::Soccer() : Cabsl<Soccer>(BehaviorOptionRegistry::theInstance->theActivationGraph)
// {
// }

Soccer::Soccer() : Cabsl<Soccer>(&activationGraph)
{
}

void Soccer::execute()
{
    beginFrame(theFrameInfo.time);
    Cabsl<Soccer>::execute(OptionInfos::getOption("Root"));
    endFrame();
}