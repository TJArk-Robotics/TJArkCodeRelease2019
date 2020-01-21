#pragma once

#include <cstdint>
#include <cstring>

namespace RoboCup
{
#define teamColour teamColor
#include "GameController/RoboCupGameControlData.h"
#include "GameController/SPLStandardMessage.h"
#undef teamColour
} // namespace RoboCup
