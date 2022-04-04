//
// Created by admin on 04.04.2022.
//

#ifndef TRANSFORMATIONS_MATRIX4X3_H
#define TRANSFORMATIONS_MATRIX4X3_H

// forward declaration
class Vector3;
class EulerAngles;
class Quaternion;
class RotationMatrix;

// Implement a 4x3 transformation matrix.
// This class can represent any affine transformation
class Matrix4x3 {
public:
    // The values of the matrix
    // Basically the upper 3x3 portion contains a linear transformation,
    // the last row is the translation portion.
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;
    float tx, ty, tz;

    // Set to identity
    void identity();

    // Access the translation portion of the matrix directly
    void zeroTranslation();
    void setTranslation(Vector3 const& v);
    void setupTranslation(Vector3 const& v);

    // Setup the matrix to perform a specific transform from
    // parent->local space, assuming the local space is in the specific position
    // and orientation within the parent space.
    // The orientation may be specified using Euler angles or a rotation matrix.
    void setupLocalToParent(Vector3 const& pos, EulerAngles const& orient);
    void setupLocalToParent(Vector3 const& pos, RotationMatrix const& orient);
    void setupParentToLocal(Vector3 const& pos, EulerAngles const& orient);
    void setupParentToLocal(Vector3 const& pos, RotationMatrix const& orient);

    // Setup the matrix to perform a rotation about a cardinal axis
    void setupRotate(int axis, float theta);

    // Setup the matrix to perform a rotation about an arbitrary axis
    void setupRotate(Vector3 const& axsi, float theta);

    // Setup the matrix to perform a rotation,
    // given the angular displacement in quaternion form
    void fromQuaternion(Quaternion const& q);

    // Setup the matrix to perform scale on each axis
    void setupScale(Vector3 const& s);

    // Setup the matrix to perform scale along an arbitrary axis
    void setupScaleAlongAxis(Vector3 const& axis, float k);

    // Setup the matrix to perform a shear
    void setupShear(int axis, float s, float t);

    // Setup the matrix to perform a projection
    // onto a plane passing through the origin
    void setupProject(Vector3 const& n);

    // Setup the matrix to perform a reflection about a plane parallel
    // to a cardinal plane
    void setupReflect(int axis, float k = 0.0f);

    // Setup the matrix to perform a reflection about an
    // arbitrary plane  through the origin
    void setupReflect(Vector3 const& n);

private:

};

// Operator* is used to transform a point and for concatenates matrices
Vector3 operator*(Vector3 const& p, Matrix4x3 const& m);
Matrix4x3 operator*(Matrix4x3 const& a, Matrix4x3 const& b);
Vector3& operator*=(Vector3 const& p, Matrix4x3 const& m);
Matrix4x3& operator*=(Matrix4x3 const& a, Matrix4x3 const& b);

// Compute the determinant of the 3x3 portion
float determinant(Matrix4x3 const& m);

// Compute the inverse of a matrix
Matrix4x3 inverse(Matrix4x3 const& m);

// Extract the translation portion of the matrix
Vector3 getTranslation(Matrix4x3 const& m);

// Extract the position/orientation from a local->parent matrix,
// or parent->local matrix
Vector3 getPositionFromParentToLocalMatrix(Matrix4x3 const& m);
Vector3 getPositionFromLocalToParentMatrix(Matrix4x3 const& m);

#endif //TRANSFORMATIONS_MATRIX4X3_H
