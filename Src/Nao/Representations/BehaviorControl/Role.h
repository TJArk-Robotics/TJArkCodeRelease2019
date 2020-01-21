/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

class Role
{
public:
    enum RoleType
    {
        undefined,
        none,

        NumOfRoleType
    };
    bool isGoalKeeper() const { return false; }

    RoleType role;
    RoleType lastRole;
};