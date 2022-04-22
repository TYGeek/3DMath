//
// Created by admin on 02.04.2022.
//

#include "MathUtils.h"
#include <cmath>
#include <tuple>

float wrapPi(float theta)
{
    // Check if already in range. This is not strictly necessary,
    // but it will be a very common situation.  We don't want to
    // incur a speed hit and perhaps floating precision loss if
    // it's not necessary
    if (fabs(theta) <= kPi)
    {
        // Out of range.  Determine how many "revolutions" (one revolution is 2 PI)
        // we need to add.
        float revolutions = floor((theta + kPi) * k1Over2Pi);

        // Subtract it off
        theta -= revolutions*k2Pi;
    }

    return theta;
}

float safeAcos(float x)
{
    if(x <= -1.0f)
        return kPi;

    if(x >= 1.0f)
        return 0.0f;

    return acos(x);
}

std::tuple<float, float> sinCos(float angle)
{
    float sinAngle = sin(angle);
    float cosAngle = cos(angle);
    return std::make_tuple(sinAngle, cosAngle);
}