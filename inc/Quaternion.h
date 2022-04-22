//
// Created by admin on 02.04.2022.
//

#ifndef TRANSFORMATIONS_QUATERNION_H
#define TRANSFORMATIONS_QUATERNION_H

// forward declaration
class Vector3;
class EulerAngles;
class RotationMatrix;

// implement a quaternion for the purpose of
// representing an angular displacement (orientation) in 3D
class Quaternion {
public:
    // the four values of quaternion
    float w, x, y, z;

    Quaternion():w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f}{};
    Quaternion(float w, float x, float y, float z): w{w}, x{x}, y{y}, z{z}{};
    ~Quaternion() = default;

    //set to identity
    void identity();

    // setup the quaternion to a specific rotation
    void setToRotateAboutX(float angle);
    void setToRotateAboutY(float angle);
    void setToRotateAboutZ(float angle);
    void setToRotateAboutAxis(const Vector3& axis, float angle);

    // Setup the quaternion from the rotation matrix
    void fromInertialToObjectMatrix(RotationMatrix const& q);

    // setup to perform object<->inertial rotation,
    // given orientation in Euler angle format
    void setRotateObjectToInertial(const EulerAngles& orientation);
    void setRotateInertialToObject(const EulerAngles& orientation);

    // cross product
    Quaternion operator*(const Quaternion& q) const;
    Quaternion& operator*=(const Quaternion& q);

    // normalize quaternion
    void normalize();

    // extract the rotation axis and angle
    float getRotationAngle() const;
    Vector3 getRotationAxis() const;

private:


};
 // global identity quaternion constant
 extern const Quaternion kQuaternionIdentity;

 // quaternion dot product
 extern float dotProduct(Quaternion const& q1, Quaternion const& q2);

 // spherical linear interpolation
 extern Quaternion slert(Quaternion const& q0, Quaternion const& q1, float t);

 // quaternion conjugation
 extern Quaternion conjugate(Quaternion const& q);

 // quaternion exponentiation
 extern Quaternion pow(Quaternion const& q, float exponent);

#endif //TRANSFORMATIONS_QUATERNION_H
