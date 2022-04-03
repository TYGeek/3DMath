//
// Created by admin on 02.04.2022.
//
#include <cmath>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "mathUtils.h"

// global Euler angles identity
// we do not know exactly when this object can be constructed in relation to other object,
// so it is possible for the object to be referenced before it is initialized.
// It will be zero-initialized at program startup.
const EulerAngles kEulerAnglesIdentity{0.0f, 0.0f, 0.0f};

void EulerAngles::identity() {
    heading = 0.0f;
    pitch = 0.0f;
    bank = 0.0f;
}

void EulerAngles::canonize() {
    // wrap pitch in range -pi...pi
    pitch = wrapPi(pitch);

    // check the matrix pitch outside the canonical range of -pi/2...pi/2
    if(pitch < -kPiOver2)
    {
        pitch = -kPi - pitch;
        heading += kPi;
        bank += kPi;
    } else if(pitch > kPiOver2)
    {
        pitch = kPi - pitch;
        heading += kPi;
        bank += kPi;
    }

    // check for the Gimbal lock
    if(std::fabs(pitch) > kPiOver2 - 1e-4)
    {
        // we are in Gimbal lock all rotation about vertical axis to heading
        heading += bank;
        bank = 0.0f;
    } else
    {
        // not in Gimbal lock
        // wrap the angle in canonical range
        bank = wrapPi(bank);
    }

    // wrap heading in canonical range
    heading = wrapPi(heading);
}

// Setup Euler angle, given an object->inertial rotation quaternion
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
    // extract sin pitch
    float sp = -2.0f * (q.y * q.z - q.w * q.x);

    // check for Gimbal lock
    if(std::fabs(sp) > 0.9999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;

        // compute heading, slam bank to zero
        heading = std::atan2(-q.x*q.z + q.w*q.y,
                             0.5f - q.y*q.y - q.z*q.z);

        bank = 0.0f;
    }
    else
    {
        // compute angles.
        // we don't have to use the safe asin()
        // because we already checked for range error
        // when checking for Gimbal lock
        pitch = asin(sp);

        heading = std::atan2(q.x*q.z + q.w*q.y,
                             0.5f - q.x*q.x - q.y*q.y);

        bank = std::atan2(q.x*q.y + q.w*q.z,
                          0.5f - q.x*q.x - q.z*q.z);
    }
}

// Setup Euler angle, given an inertial->object rotation quaternion
void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q) {
    // extract sin pitch
    float sp = -2.0f * (q.y * q.z + q.w * q.x);

    // check for Gimbal lock
    if(std::fabs(sp) > 0.9999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;

        // compute heading, slam bank to zero
        heading = std::atan2(-q.x*q.z - q.w*q.y,
                             0.5f - q.y*q.y - q.z*q.z);

        bank = 0.0f;
    }
    else
    {
        // compute angles.
        // we don't have to use the safe asin()
        // because we already checked for range error
        // when checking for Gimbal lock
        pitch = asin(sp);

        heading = std::atan2(q.x*q.z - q.w*q.y,
                             0.5f - q.x*q.x - q.y*q.y);

        bank = std::atan2(q.x*q.y - q.w*q.z,
                          0.5f - q.x*q.x - q.z*q.z);
    }
}

// setup Euler angle, given an object->world transformation matrix
void EulerAngles::fromObjectToWorldMatrix(const Matrix4X3 &m) {
    // extract sin(pitch) from m32
    float sp = -m.m32;

    // check for Gimbel lock
    if(fabs(sp) > 9.99999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;
        // compute heading, pitch bank to zero
        heading = atan2(-m.m23, m.m11);
        bank = 0.0f;
    }
    else
    {
        // compute angles
        heading = atan2(m.m31, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m12, m.m22);
    }
}

// setup Euler angle, given an world->object transformation matrix
void EulerAngles::fromWorldToObjectMatrix(const Matrix4X3 &m) {
    // extract sin(pitch) from m23
    float sp = -m.m23;

    // check for Gimbel lock
    if(fabs(sp) > 9.99999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;
        // compute heading, pitch bank to zero
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    }
    else
    {
        // compute angles
        heading = atan2(m.m13, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m21, m.m22);
    }
}

// setup an Euler angle, given an rotation matrix
void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
    // extract sin(pitch) from m23
    float sp = -m.m23;

    // check for Gimbel lock
    if(fabs(sp) > 9.99999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;
        // compute heading, pitch bank to zero
        heading = atan2(-m.m31, m.m11);
        bank = 0.0f;
    }
    else
    {
        // compute angles
        heading = atan2(m.m13, m.m33);
        pitch = asin(sp);
        bank = atan2(m.m21, m.m22);
    }
}
