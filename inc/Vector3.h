#ifndef VECTOR3_VECTOR3_H
#define VECTOR3_VECTOR3_H

#include <ostream>

class Vector3
{
public:
    // Variables
    float x, y, z;

    // Constructors
    Vector3();
    Vector3(Vector3 const& vec);
    Vector3(float x, float y, float z);

    // Assigment operation
    Vector3& operator=(Vector3 const& vec);

    // Logical operation
    bool operator==(Vector3 const& vec) const;
    bool operator!=(Vector3 const& vec) const;

    // Set the vector to zero
    void zero();
    // Unary minus (return negative of the Vector)
    Vector3 operator-() const;
    // Binary + add operation
    Vector3 operator+(Vector3 const& vec) const;
    Vector3& operator+=(Vector3 const& vec);
    // Binary - subtract operation
    Vector3 operator-(Vector3 const& vec) const;
    Vector3& operator-=(Vector3 const& vec);
    // Multiplication by scalar
    Vector3 operator*(float scalar) const;
    Vector3& operator*=(float scalar);
    // Division by scalar
    Vector3 operator/(float scalar) const;
    Vector3& operator/=(float scalar);
    // Vector dot product
    float operator*(Vector3 const& vec) const;
    // Normalize the vector
    void normalize();

private:

    // Friend function calculate multiplication scalar by vector
    friend Vector3 operator*(float scalar, Vector3 const& vec);
    // Friend function cross product of two vectors
    friend Vector3 crossProduct(Vector3 const& lhs, Vector3 const& rhs);
    // Friend function calculate magnitude of a vector
    friend float vectorMag(Vector3 const& vec);
    // Friend function compute distance between two vectors
    friend float distance(Vector3 const& a, Vector3 const& b);
    friend std::ostream& operator<<(std::ostream& os, Vector3 const& vec);
};

Vector3 operator*(float scalar, Vector3 const& vec);
Vector3 crossProduct(Vector3 const& lhs, Vector3 const& rhs);
float vectorMag(Vector3 const& vec);
float distance(Vector3 const& a, Vector3 const& b);
std::ostream& operator<<(std::ostream& os, Vector3 const& vec);

#endif //VECTOR3_VECTOR3_H
