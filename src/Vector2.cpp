//
// Created by admin on 27.04.2022.
//

#include <cmath>
#include <ostream>
#include "Vector2.h"

Vector2::Vector2():x{0.0}, y{0.0} { }

Vector2::Vector2(const Vector2 &vec):x{vec.x}, y{vec.y} { }

Vector2::Vector2(float x, float y):x{x}, y{y} { }

Vector2 &Vector2::operator=(const Vector2 &vec) {
    x = vec.x;
    y = vec.y;
    return *this;
}

bool Vector2::operator==(const Vector2 &vec) const {
    return x == vec.x && y == vec.y;
}

bool Vector2::operator!=(const Vector2 &vec) const {
    return x != vec.x && y != vec.y;
}

void Vector2::zero() {
    x = 0.0;
    y = 0.0;
}

Vector2 Vector2::operator-() const {
    return {-x, -y};
}

Vector2 Vector2::operator+(const Vector2 &vec) const {
    return {x + vec.x, y + vec.y};
}

Vector2 &Vector2::operator+=(const Vector2 &vec) {
    x += vec.x;
    y += vec.y;
    return *this;
}

Vector2 Vector2::operator-(const Vector2 &vec) const {
    return {x-vec.x, y-vec.y};
}

Vector2 &Vector2::operator-=(const Vector2 &vec) {
    x -= vec.x;
    y -= vec.y;
    return *this;
}

Vector2 Vector2::operator*(float scalar) const {
    return {scalar*x, scalar*y};
}

Vector2 &Vector2::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2 Vector2::operator/(float scalar) const {
    float oneOverScalar = 1 / scalar; // No check zero division!
    return {x*oneOverScalar, y*oneOverScalar};
}

Vector2 &Vector2::operator/=(float scalar) {
    float oneOverScalar = 1 / scalar; // No check zero division!
    x *= oneOverScalar;
    y *= oneOverScalar;
    return *this;
}

float Vector2::operator*(const Vector2 &vec) const {
    return x*vec.x + y*vec.y;
}

void Vector2::normalize() {
    float magSq = x*x + y*y;
    if(magSq > 0)
    {
        float oneOverMag = 1.0f / sqrt(magSq);
        x *= oneOverMag;
        y *= oneOverMag;
    }
}

Vector2 operator*(float scalar, const Vector2 &vec) {
    return {vec.x*scalar, vec.y*scalar};
}

float crossProduct(const Vector2 &lhs, const Vector2 &rhs) {
    return lhs.x*rhs.y - lhs.y*rhs.x;
}

float vectorMag(const Vector2 &vec) {
    float magSq = vec.x*vec.x + vec.y*vec.y;

    if(magSq > 0)
        return sqrt(magSq);
    else
        return 0;
}

float distance(const Vector2 &a, const Vector2 &b) {
    Vector2 c = b - a;
    return vectorMag(c);
}

std::ostream &operator<<(std::ostream &os, const Vector2 &vec) {
    os << "[" << vec.x << ", " << vec.y << "]";
    return os;
}
