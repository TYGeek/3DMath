//
// Created by admin on 02.04.2022.
//
#include <cmath>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"
#include "MathUtils.h"

// The global Euler angles identity constant.
// We do not know exactly when this object can be constructed in relation to other object,
// so it is possible for the object to be referenced before it is initialized.
// It will be zero-initialized at program startup.
const EulerAngles kEulerAnglesIdentity{0.0f, 0.0f, 0.0f};

void EulerAngles::identity() {
    heading = 0.0f;
    pitch = 0.0f;
    bank = 0.0f;
}

// Set the Euler angle triple to its canonical value.
// This does not change the meaning of the Euler angles as representation of
// orientation in 3D, but if the angles are for other purposes
// as angular velocities, etc., then the operation might not be valid.
// See 10.3.4
void EulerAngles::canonize() {
    // wrap pitch in range -pi...pi
    pitch = wrapPi(pitch);

    // check the matrix pitch outside the canonical range of -pi/2...pi/2
    if(pitch < -kPiOver2)
    {
        pitch = -kPi - pitch;
        heading += kPi;
        bank += kPi;
    }
    else if(pitch > kPiOver2)
    {
        pitch = kPi - pitch;
        heading += kPi;
        bank += kPi;
    }

    // check for the Gimbal lock, value near -Pi/2 or +Pi/2
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
// See 10.6.6
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
    // extract sin(pitch)
    float sp = -2.0f * (q.y * q.z - q.w * q.x);

    // check for Gimbal lock
    if(std::fabs(sp) > 0.999f)
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
// See 10.6.6
void EulerAngles::fromInertialToObjectQuaternion(const Quaternion &q) {
    // extract sin pitch
    float sp = -2.0f * (q.y * q.z + q.w * q.x);

    // check for Gimbal lock
    if(std::fabs(sp) > 0.999f)
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

// Setup Euler angle, given an object->world transformation matrix
// The matrix assumed to be orthogonal.
// The translation portion is ignored.
// M_object->inertial = (M_inertial->object)^-1 = B^-1*P^-1*H^-1
//                    = Rz(−b) Rx(−p) Ry(−h) = { ch*cb+sh*sp*sb    sb*cp    −sh*cb+ch*sp*sb
//                                              −ch*sb+sh*sp*cb    cb*cp     sb*sh+ch*sp*cb
//                                                    sh*cp         -sp           ch*cp    }
// See 10.6.2
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m) {
    // extract sin(pitch) from m32
    float sp = -m.m32;

    // check for Gimbel lock
    if(fabs(sp) > 0.999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;
        // compute heading, pitch bank to zero
        bank = 0.0f;
        heading = atan2(-m.m23, m.m11);
    }
    else
    {
        // compute angles
        heading = atan2(m.m31, m.m33);
        // asic() return value in the range -Pi/2...+Pi/2
        pitch = asin(sp);
        bank = atan2(m.m12, m.m22);
    }
}

// Setup Euler angle, given a world->object transformation matrix
// The matrix assumed to be orthogonal.
// The translation portion is ignored.
// M_inertial->object = HPB = Ry(−h) Rx(−p) Rz(−b) = { ch*cb+sh*sp*sb   −ch*sb+sh*sp*cb    sh*cp
//                                                          sb*cp           cb*cp           −sp
//                                                     −sh*cb+ch*sp*sb   sb*sh+ch*sp*cb    ch*cp }
// See 10.6.2
void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m) {
    // extract sin(pitch) from m23
    float sp = -m.m23;

    // check for Gimbal lock
    if(fabs(sp) > 0.999f)
    {
        // looking straight up or down
        pitch = kPiOver2 * sp;
        // compute heading, pitch bank to zero
        bank = 0.0f;
        heading = atan2(-m.m31, m.m11);
    }
    else
    {
        // compute angles
        // asic() return value in the range -Pi/2...+Pi/2
        pitch = asin(sp);
        // atan2() return value in the range -Pi...+Pi
        heading = atan2(m.m13, m.m33);
        bank = atan2(m.m21, m.m22);
    }
}

// Setup an Euler angle, given a rotation matrix
// See 10.6.2
void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
    // extract sin(pitch) from m23
    float sp = -m.m23;

    // check for Gimbal lock
    if(fabs(sp) > 0.999f)
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
        pitch = asin(sp);
        heading = atan2(m.m13, m.m33);
        bank = atan2(m.m21, m.m22);
    }
}

std::ostream& operator<<(std::ostream& os, EulerAngles const& orient)
{
    os << "{ h: " << orient.heading/RAD << ", "
       <<   "p: " << orient.pitch/RAD   << ", "
       <<   "b: " << orient.bank/RAD    << " }";
    return os;
}