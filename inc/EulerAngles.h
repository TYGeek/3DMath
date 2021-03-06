#ifndef TRANSFORMATIONS_EULERANGLES_H
#define TRANSFORMATIONS_EULERANGLES_H

#include <ostream>

/*  Left-handed coordinate space   *
 * * * * * * * * * * * * * * * * * *
 *                                 *
 *         | y+ (1. heading)       *
 *         |                       *
 *         |________ z+ (3. bank)  *
 *        /                        *
 *       /                         *
 *      / x+ (2. pitch)            *
 *                                 *
 * * * * * * * * * * * * * * * * * *
 */

// Forward declaration
class Quaternion;
class Matrix4x3;
class RotationMatrix;

// This class is used to store an orientation that
// represent heading->pitch->bank Euler angle triple
// TODO: implement operation addition and subtraction, multiplication by scalar etc.
//       implement move constructor
class EulerAngles {
public:
    float heading;
    float pitch;
    float bank;

    EulerAngles();
    EulerAngles(float h, float p, float b);
    ~EulerAngles() = default;

    // Set identity triple (all zeros)
    void identity();

    // Determine canonical Euler angle triple
    void canonize();

    // Convert Quaternion to Euler angle format
    void fromObjectToInertialQuaternion(Quaternion const& q);
    void fromInertialToObjectQuaternion(Quaternion const& q);

    // Convert the transform matrix to Euler angle format.
    // The translation portion of the matrix is ignored.
    // The matrix is assumed to be orthogonal
    void fromObjectToWorldMatrix(Matrix4x3 const& m);
    void fromWorldToObjectMatrix(Matrix4x3 const& m);

    // Convert a rotation matrix to Euler angle form.
    void fromRotationMatrix(RotationMatrix const& m);

private:

};

std::ostream& operator<<(std::ostream& os, EulerAngles const& vec);

// a global identity Euler angle constant
extern const EulerAngles kEulerAnglesIdentity;

#endif //TRANSFORMATIONS_EULERANGLES_H
