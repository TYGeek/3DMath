//
// Created by admin on 02.04.2022.
//

#include "mathUtils.h"
#include <cmath>
#include "tuple"


float wrapPi(float angle)
{
    angle += kPi;
    angle -= floor(angle * k1Over2Pi) * k2Pi;
    angle -= kPi;
    return angle;
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