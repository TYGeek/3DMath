//
// Created by admin on 27.04.2022.
//

#ifndef TRANSFORMATIONS_VECTOR2_H
#define TRANSFORMATIONS_VECTOR2_H

#include <ostream>

class Vector2
{
public:
    // Variables
    float x, y;

    // Constructors
    Vector2();
    Vector2(Vector2 const& vec);
    Vector2(float x, float y);

    // Assigment operation
    Vector2& operator=(Vector2 const& vec);

    // Logical operation
    bool operator==(Vector2 const& vec) const;
    bool operator!=(Vector2 const& vec) const;

    // Set the vector to zero
    void zero();
    // Unary minus (return negative of the Vector)
    Vector2 operator-() const;
    // Binary + add operation
    Vector2 operator+(Vector2 const& vec) const;
    Vector2& operator+=(Vector2 const& vec);
    // Binary - subtract operation
    Vector2 operator-(Vector2 const& vec) const;
    Vector2& operator-=(Vector2 const& vec);
    // Multiplication by scalar
    Vector2 operator*(float scalar) const;
    Vector2& operator*=(float scalar);
    // Division by scalar
    Vector2 operator/(float scalar) const;
    Vector2& operator/=(float scalar);
    // Vector dot product
    float operator*(Vector2 const& vec) const;
    // Normalize the vector
    void normalize();

private:

    // Friend function calculate multiplication scalar by vector
    friend Vector2 operator*(float scalar, Vector2 const& vec);
    // Friend function cross product of two vectors
    /// The cross product of 2D vectors results in a 3D vector with only a z-component.
    /// This function returns the magnitude of the z-value.
    friend float crossProduct(Vector2 const& lhs, Vector2 const& rhs);
    // Friend function calculate magnitude of a vector
    friend float vectorMag(Vector2 const& vec);
    // Friend function compute distance between two vectors
    friend float distance(Vector2 const& a, Vector2 const& b);
    friend std::ostream& operator<<(std::ostream& os, Vector2 const& vec);
};

Vector2 operator*(float scalar, Vector2 const& vec);
float crossProduct(Vector2 const& lhs, Vector2 const& rhs);
float vectorMag(Vector2 const& vec);
float distance(Vector2 const& a, Vector2 const& b);
std::ostream& operator<<(std::ostream& os, Vector2 const& vec);


#endif //TRANSFORMATIONS_VECTOR2_H
