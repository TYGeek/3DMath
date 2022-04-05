//
// Created by admin on 03.04.2022.
//

#ifndef TRANSFORMATIONS_ROTATIONMATRIX_H
#define TRANSFORMATIONS_ROTATIONMATRIX_H

// forward declaration
class Vector3;
class EulerAngles;
class Quaternion;

// Implement a simple 3x3 matrix that it used for ROTATION ONLY.
// The matrix is assumed to be orthogonal.
// The direction of transformation is specified at the time of transformation.
class RotationMatrix {
public:
    // The 9 values of matrix
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;

    RotationMatrix():   m11{1.0f},m12{0.0f},m13{0.0f},
                        m21{0.0f},m22{1.0f},m23{0.0f},
                        m31{0.0f},m32{0.0f},m33{1.0f}{}
    // Set to identity
    void identity();

    // Setup the matrix with a specific orientation
    void setup(EulerAngles const& orientation);

    // Setup the matrix from quaternion
    void fromInertialToObjectQuaternion(Quaternion const& q);
    void fromObjectToInertialQuaternion(Quaternion const& q);

    // Perform rotation
    Vector3 inertialToObject(Vector3 const& v) const;
    Vector3 objectToInertial(Vector3 const& v) const;
private:

};


#endif //TRANSFORMATIONS_ROTATIONMATRIX_H
