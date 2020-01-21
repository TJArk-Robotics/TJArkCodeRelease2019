/**
 * @file GlobalOptions.h
 *
 * This file defines a representation that stores global options. These might
 * be specified per scenario or per location, but not per robot.
 *
 * @author Thomas Röfer
 */
#pragma once

class GlobalOptions
{
public:
    bool slowWalk = false; /**< Use the slow walking parameters for all robots. */
};