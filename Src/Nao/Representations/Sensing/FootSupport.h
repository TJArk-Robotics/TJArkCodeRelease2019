#pragma once

class FootSupport
{
public:
    float support = 0.f;   /** Unitless distribution of the support over both feet (left - right). */
    bool switched = false; /** The support foot switched. */
};