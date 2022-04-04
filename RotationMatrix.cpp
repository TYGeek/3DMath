//
// Created by admin on 03.04.2022.
//

#include "RotationMatrix.h"
#include "vector3.h"
#include "mathUtils.h"
#include "Quaternion.h"
#include "EulerAngles.h"

// Set the matrix to the identity matrix
void RotationMatrix::identity() {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
}

// Setup the matrix with the specific orientation
void RotationMatrix::setup(const EulerAngles &orientation) {
    // Fetch sine and cosine of angles
    float sh, ch;
    float sp, cp;
    float sb, cb;

    std::tie(sh, ch) = sinCos(orientation.heading);
    std::tie(sp, cp) = sinCos(orientation.pitch);
    std::tie(sb, cb) = sinCos(orientation.bank);

    // Fill in the matrix element
    m11 = ch*cb + sh*sp*sb;
    m12 = -ch*sb + sh*sp*cb;
    m13 = sh*cp;

    m21 = sb*cp;
    m22 = cb*cp;
    m23 = -sp;

    m31 = -sh*cb + ch*sp*sb;
    m32 = sb*sh + ch*sp*cb;
    m33 = ch*cp;
}

// Setup the matrix,
// given a quaternion that performs an inertial->object rotation
void RotationMatrix::fromInertialToObjectQuaternion(const Quaternion &q) {
    // Fill in the matrix elements
    m11 = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    m12 = 2.0f*(q.x*q.y + q.w*q.z);
    m13 = 2.0f*(q.x*q.z - q.w*q.y);

    m21 = 2.0f*(q.x*q.y - q.w*q.z);
    m22 = 1.0f - 2.0f*(q.x*q.x + q.z*q.z);
    m23 = 2.0f*(q.y*q.z + q.w*q.x);

    m31 = 2.0f*(q.x*q.z + q.w*q.y);
    m32 = 2.0f*(q.y*q.z - q.w*q.x);
    m33 = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
}

// Setuo the matrix,
// given a quaternion that performs an object->inertial rotation
void RotationMatrix::fromObjectToInertialQuaternion(const Quaternion &q) {
    // Fill in matrix element
    m11 = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    m12 = 2.0f*(q.x*q.y - q.w*q.z);
    m13 = 2.0f*(q.x*q.z + q.w*q.y);

    m21 = 2.0f*(q.x*q.y + q.w*q.z);
    m22 = 1.0f - 2.0f*(q.x*q.x + q.z*q.z);
    m23 = 2.0f*(q.y*q.z - q.w*q.x);

    m31 = 2.0f*(q.x*q.z - q.w*q.y);
    m32 = 2.0f*(q.y*q.z + q.w*q.x);
    m33 = 1.0f - 2.0f*(q.x*q.x + q.y*q.y);
}

// Rotate a vector from inertial to object space
Vector3 RotationMatrix::inertialToObject(const Vector3 &v)const {
    // Perform the matrix multiplication in the "standart" way
    return Vector3(m11*v.x + m21*v.y + m31*v.z,
                   m12*v.x + m22*v.y + m32*v.z,
                   m13*v.x + m23*v.y + m33*v.z);
}

// Rotate a vector from object to inertial space
Vector3 RotationMatrix::objectToInertial(const Vector3 &v) const {
    // Multiply by transpose
    return Vector3(m11*v.x + m12*v.y + m13*v.z,
                   m21*v.x + m22*v.y + m23*v.z,
                   m31*v.x + m32*v.y + m33*v.z);
}
