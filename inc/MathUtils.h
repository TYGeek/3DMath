#ifndef TRANSFORMATIONS_MATHUTILS_H
#define TRANSFORMATIONS_MATHUTILS_H
#include <utility>


// declare a global constant
const float kPi = 3.14159265f;
const float k2Pi = 2.0f * kPi;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1 / kPi;
const float k1Over2Pi = 1 / k2Pi;
const float RAD = kPi/180.0f;

// wrap an angle in range -pi...pi
float wrapPi(float theta);

// "safe" inverse trig functions
float safeAcos(float x);

// compute sin()->first and cos()->second of an angle
std::tuple<float, float> sinCos(float angle);

#endif //TRANSFORMATIONS_MATHUTILS_H
