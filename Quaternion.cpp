//
// Created by admin on 02.04.2022.
//

#include "assert.h"
#include "cmath"
#include "tuple"

#include "Quaternion.h"
#include "mathUtils.h"
#include "vector3.h"
#include "EulerAngles.h"

// global identity quaternion
// we do not know exactly when this object can be constructed in relation to other object,
// so it is possible for the object to be referenced before it is initialized.
// It will be zero-initialized at program startup.
const Quaternion kQuaternionIdentity {1.0f, 0.0f, 0.0f, 0.0f};

// Set quaternion to rotate about the specific axis
void Quaternion::setToRotateAboutX(float angle) {
    // compute the half angle
    float angleOver2 = angle * 0.5f;

    // set the values
    w = cos(angleOver2);
    x = sin(angleOver2);
    y = 0.0f;
    z = 0.0f;
}

void Quaternion::setToRotateAboutY(float angle) {
    // compute the half angle
    float angleOver2 = angle * 0.5f;

    // set the values
    w = cos(angleOver2);
    x = 0.0f;
    y = sin(angleOver2);
    z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float angle) {
    // compute the half angle
    float angleOver2 = angle * 0.5f;

    // set the values
    w = cos(angleOver2);
    x = 0.0f;
    y = 0.0f;
    z = sin(angleOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float angle) {
    // the axis of rotation must be normalized
    assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);

    // compute the half angle and its sine
    float angleOver2 = angle * 0.5f;
    float sinAngleOver2 = sin(angleOver2);

    // set values
    w = cos(angleOver2);
    x = axis.x * sinAngleOver2;
    y = axis.y * sinAngleOver2;
    z = axis.z * sinAngleOver2;
}

// Setup quaternion to perform object->inertial rotation,
// given orientation in Euler angles format
void Quaternion::setRotateObjectToInertial(const EulerAngles &orientation) {
    // compute sine and cosine of the half angles
    float sp, sb, sh;
    float cp, cb, ch;
    std::tie(sp, cp) = sinCos(orientation.pitch);
    std::tie(sb, cb) = sinCos(orientation.bank);
    std::tie(sh, ch) = sinCos(orientation.heading);

    // compute values
    w = ch*cp*cb + sh*sp*sb;
    x = ch*sp*cb + sh*cp*sb;
    y = -ch*sp*sb + sh*cp*cb;
    z = -sh*sp*cb + ch*cp*sb;
}

// Setup quaternion to perform inertial->object rotation,
// given orientation in Euler angles format
void Quaternion::setRotateInertialToObject(const EulerAngles &orientation) {
    // compute sine and cosine of the half angles
    float sp, sb, sh;
    float cp, cb, ch;
    std::tie(sp, cp) = sinCos(orientation.pitch);
    std::tie(sb, cb) = sinCos(orientation.bank);
    std::tie(sh, ch) = sinCos(orientation.heading);

    // compute values
    w = ch*cp*cb + sh*sp*sb;
    x = -ch*sp*cb - sh*cp*sb;
    y = ch*sp*sb - sh*cp*cb;
    z = sh*sp*cb - ch*cp*sb;
}

// Quaternion cross product - concatenate multiple angular displacement.
// The order of multiplication from left to right.
Quaternion Quaternion::operator*(const Quaternion &q) const {
    Quaternion result;

    result.w = w*q.w - x*q.x - y*q.y - z*q.z;
    result.x = w*q.x + x*q.w + z*q.y - y*q.z;
    result.y = w*q.y + y*q.w + x*q.z - z*q.x;
    result.z = w*q.z + z*q.w + y*q.x - x*q.y;

    return result;
}

Quaternion &Quaternion::operator*=(const Quaternion &q) {
    *this = *this * q;
    return *this;
}

// normalize a quaternion.
// this function combat floating point error creep
void Quaternion::normalize() {
    // compute magnitude of the quaternion
    float mag = sqrt(w*w + x*x + y*y + z*z);

    // protect to divide by zero
    if(mag > 0.0f)
    {
        // normalize it
        float oneOverMag = 1.0f/mag;
        w *= oneOverMag;
        x *= oneOverMag;
        y *= oneOverMag;
        z *= oneOverMag;
    }
    else
    {
        // have a problem
        assert(false);
        // in product do identity
        identity();
    }
}

// return the rotation angle
float Quaternion::getRotationAngle() const {
    // compute the half angle
    // remember that w = cos(angle/2)
    float angleOver2 = safeAcos(w);

    // return the rotation angle
    return angleOver2 * 2.0f;
}

// return the rotation axis
Vector3 Quaternion::getRotationAxis() const {
    // create valid vector in advance (due to NRVO)
    Vector3 result{1.0f, 0.0f, 0.0f};

    // compute sin^2(angle/2).
    // remember: w = cos(angle/2);
    // sin^2(x) + cos^2(x) = 1;

    float sinAngleOver2Sq = 1 - w*w;

    // protect against numerical imprecision
    if (sinAngleOver2Sq <= 0.0f)
    {
        // identity quaternion or numerical imprecision
        // just return valid vector
        return result;
    }

    // compute: 1/sin(angle/2)
    float oneOverSinAngleOver2 = 1/sqrt(sinAngleOver2Sq);

    // compute axis of rotation
    result.x = x*oneOverSinAngleOver2;
    result.y = y*oneOverSinAngleOver2;
    result.z = z*oneOverSinAngleOver2;

     // return axis of rotation
    return result;
}

void Quaternion::identity() {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

// Quaternion dot product. We use nonmember function
// so we can pass quaternion expressions as operands
float dotProduct(Quaternion const& q1, Quaternion const& q2)
{
    return q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
}

// Spherical linear interpolation (slert)
Quaternion slert(Quaternion const& q0, Quaternion const& q1, float t)
{
    // check for out of range parameter
    if(t <= 0.0f) return q0;
    if(t >= 1.0f) return q1;

    // compute cosine angle between quaternion using dot product
    float cosOmega = dotProduct(q0, q1);

    // if negate dot use -q1.
    // quaternion q and -q represent the same rotation, but may produce different slert.
    // we chose q or -q to rotate using the acute angle
    float q1w = q1.w;
    float q1x = q1.x;
    float q1y = q1.y;
    float q1z = q1.z;
    if(cosOmega < 0.0f)
    {
        q1w = -q1w;
        q1x = -q1x;
        q1y = -q1y;
        q1z = -q1z;
        cosOmega = -cosOmega;
    }

    // we should have two unit quaternion, so dot should be <= 1.0
    assert(cosOmega < 1.1f);

    // compute interpolation fraction
    float k0, k1;
    if(cosOmega > 0.9999f)
    {
        // very close - use linear interpolation
        // which will protect again a divide by zero
        k0 = 1.0f - t;
        k1 = t;
    }
    else
    {
        // compute the sin of angle using trig identity:
        // sin^2(omega) + cos^2(omega) = 1;
        float sinOmega = sqrt(1.0f - cosOmega*cosOmega);

        // compute the angle from its sin and cos
        float omega = atan2(sinOmega, cosOmega);

        // compute the inverse of denominator
        float oneOverSinOmega = 1.0f/sinOmega;

        // compute interpolation parameter
        k0 = sin((1.0f - t) * omega) * oneOverSinOmega;
        k1 = sin(t * omega) * oneOverSinOmega;
    }

    // interpolate the result quaternion
    Quaternion result;
    result.w = k0*q0.w + k1*q1.w;
    result.x = k0*q0.x + k1*q1.x;
    result.y = k0*q0.y + k1*q1.y;
    result.z = k0*q0.z + k1*q1.z;

    return result;
}

// quaternion conjugation. this is the quaternion with the opposite rotation
Quaternion conjugate(Quaternion const& q)
{
    Quaternion result;

    // same rotation amount
    result.w = q.w;

    // opposite axis of rotation
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;

    return result;
}

// quaternion exponentiation
extern Quaternion pow(Quaternion const& q, float exponent)
{
    // check for the case of an identity quaternion.
    // This will protect against divide by zero
    if(fabs(q.w) > 0.9999f)
    {
        return q;
    }

    // extract the half angle alpha (alpha = theta/2)
    float alpha = acos(q.w);

    // compute the new alpha
    float newAlpha = alpha * exponent;

    // compute new quaternion
    Quaternion result;
    // compute new w value
    result.w = cos(newAlpha);

    // compute new xyz values
    float k = sin(newAlpha)/sin(alpha);
    result.x = q.x * k;
    result.y = q.y * k;
    result.z = q.z * k;

    return result;
}
