#include "Vector3.h"
#include <cmath>

Vector3::Vector3() {}
Vector3::Vector3(Vector3 const& vec): x{vec.x}, y{vec.y}, z{vec.z}{}
Vector3::Vector3(float x, float y, float z): x{x}, y{y}, z{z} {}

// Assigment operation
Vector3& Vector3::operator=(Vector3 const& vec)
{
    x = vec.x;
    y = vec.y;
    z = vec.z;

    return *this;
}

// Logical operation
bool Vector3::operator==(const Vector3 &vec) const
{
    return x == vec.x && y == vec.y && z == vec.z;
}

bool Vector3::operator!=(const Vector3 &vec) const
{
    return x != vec.x || y != vec.y || z != vec.z;
}

// Set zero vector
void Vector3::zero()
{
    x = 0.0; y = 0.0; z = 0.0;
}

// Unary -
Vector3 Vector3::operator-() const
{
    return {-x, -y, -z};
}

// Binary +
Vector3 Vector3::operator+(const Vector3 &vec) const
{
    return {x+vec.x, y+vec.y, z+vec.z};
}

Vector3& Vector3::operator+=(const Vector3 &vec)
{
    x += vec.x;
    y += vec.y;
    z += vec.z;
    return *this;
}

// Binary -
Vector3 Vector3::operator-(const Vector3 &vec) const
{
    return {x-vec.x, y-vec.y, z-vec.z};
}

Vector3& Vector3::operator-=(const Vector3 &vec)
{
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;
    return *this;
}

// Multiplication by scalar
Vector3 Vector3::operator*(float scalar) const
{
    return Vector3(x*scalar, y*scalar, z*scalar);
}

Vector3& Vector3::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

// Division by scalar
Vector3 Vector3::operator/(float scalar) const
{
    float oneOverScalar = 1 / scalar; // No check zero division!
    return Vector3(x*oneOverScalar, y*oneOverScalar, z*oneOverScalar);
}

Vector3& Vector3::operator/=(float scalar)
{
    float oneOverScalar = 1 / scalar; // No check zero division!
    *this *= oneOverScalar;
    return *this;
}

// Vector dot product
float Vector3::operator*(const Vector3 &vec) const
{
    return x*vec.x + y*vec.y + z*vec.z;
}

// Normalize
void Vector3::normalize()
{
    float magSq = x*x + y*y + z*z;
    if(magSq > 0)
    {
        float oneOverMag = 1.0f / sqrt(magSq);
        x *= oneOverMag;
        y *= oneOverMag;
        z *= oneOverMag;
    }
}

Vector3 operator*(float scalar, Vector3 const& vec)
{
    return Vector3(scalar*vec.x, scalar*vec.y, scalar*vec.z);
}

float vectorMag(Vector3 const& vec)
{
    float magSq = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;

    if(magSq > 0)
        return sqrt(magSq);
    else
        return 0;
}

Vector3 crossProduct(Vector3 const& lhs, Vector3 const& rhs)
{
    return Vector3(lhs.y*rhs.z - lhs.z*rhs.y,
                   lhs.z*rhs.x - lhs.x*rhs.z,
                   lhs.x*rhs.y - lhs.y*rhs.x);
}

float distance(Vector3 const& a, Vector3 const& b)
{
    Vector3 c = b - a;
    return vectorMag(c);
}

std::ostream& operator<<(std::ostream& os, Vector3 const& vec)
{
    os << "[" << vec.x << ", " << vec.y << ", " << vec.z << "]";
    return os;
}