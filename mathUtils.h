//
// Created by admin on 02.04.2022.
//

#ifndef TRANSFORMATIONS_MATHUTILS_H
#define TRANSFORMATIONS_MATHUTILS_H
#include <utility>


// declare a global constant
const float kPi = 3.14159265f;
const float k2Pi = 2.0f * kPi;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1 / kPi;
const float k1Over2Pi = 1 / k2Pi;

// wrap an angle in range -pi...pi
extern float wrapPi(float angle);

// "safe" inverse trig functions
extern float safeAcos(float x);

// compute sin()->first and cos()->second of an angle
extern std::tuple<float, float> sinCos(float angle);

#endif //TRANSFORMATIONS_MATHUTILS_H
