#include <iostream>
#include "Vector3.h"
#include "Vector2.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "RotationMatrix.h"
#include "Matrix4x3.h"
#include "MathUtils.h"


int main()
{
    // Init vector
    Vector3 vec{1.0f, 1.0f, 0.0f};

    // Set orientation in Euler angle format
    EulerAngles orientZSK{0.0f*RAD, 0.0f*RAD, 0.0f*RAD};

    // Set orientation in Quaternion format
    Quaternion q;
    q.setRotateObjectToInertial(orientZSK);

    // Set rotation matrix
    RotationMatrix matrixEuler;
    matrixEuler.setup(orientZSK);
    Vector3 r1 = matrixEuler.objectToInertial(vec);

    RotationMatrix matrixQuaternion;
    matrixQuaternion.fromObjectToInertialQuaternion(q);
    Vector3 r2 = matrixQuaternion.objectToInertial(vec);

    std::cout << vec << std::endl;
    std::cout << r1 << std::endl;
    std::cout << r2 << std::endl;

    return 0;
}
