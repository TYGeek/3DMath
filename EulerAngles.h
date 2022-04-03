//
// Created by admin on 02.04.2022.
//

#ifndef TRANSFORMATIONS_EULERANGLES_H
#define TRANSFORMATIONS_EULERANGLES_H

// Forward declaration
class Quaternion;
class Matrix4X3;
class RotationMatrix;

// This class represent heading->pitch->bank Euler angle triple
class EulerAngles {
public:
    float heading;
    float pitch;
    float bank;

    EulerAngles() = default;
    EulerAngles(float h, float p, float b): heading{h}, pitch{p}, bank{b}{};
    ~EulerAngles() = default;

    // set identity triple (all zeros)
    void identity();

    // determine canonical Euler angle triple
    void canonize();

    // convert Quaternion to Euler angle format
    void fromObjectToInertialQuaternion(Quaternion const& q);
    void fromInertialToObjectQuaternion(Quaternion const& q);

    // convert the transform matrix to Euler angle format
    // the translation portion of the matrix is ignored - matrix orthogonal
    void fromObjectToWorldMatrix(Matrix4X3 const& m);
    void fromWorldToObjectMatrix(Matrix4X3 const& m);

    // convert a rotation matrix to Euler angle format
    void fromRotationMatrix(RotationMatrix const& m);

private:

};

// a global identity Euler angle constant
extern const EulerAngles kEulerAnglesIdentity;

#endif //TRANSFORMATIONS_EULERANGLES_H
