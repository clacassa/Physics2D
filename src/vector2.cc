#include <cmath>
#include "vector2.h"

double Vector2::norm() const {
    return sqrt(x * x + y * y);
}

Vector2 Vector2::normal() const {
    const double m_norm(norm());
    if (m_norm == 0)
        return {0, 0};
    return {y / m_norm, -x / m_norm};
}

Vector2 Vector2::normalized() const {
    const double m_norm(norm());
    if (m_norm == 0)
        return {0, 0};
    return {x / m_norm, y / m_norm};
}

const Vector2 Vector2::operator+(const Vector2& v) const {
    return Vector2{x + v.x, y + v.y};
}

const Vector2 Vector2::operator-(const Vector2& v) const {
    return Vector2{x - v.x, y - v.y};
}

const Vector2 Vector2::operator*(const double a) const {
    return Vector2{x * a, y * a};
}

const Vector2 Vector2::operator/(const double a) const {
    return Vector2{x / a, y / a};
}

Vector2& Vector2::operator+=(const Vector2& v) {
     *this = *this + v;
     return *this;
}

Vector2& Vector2::operator-=(const Vector2& v) {
    *this = *this - v;
    return *this;
}

Vector2& Vector2::operator*=(const double a) {
    *this = *this * a;
    return *this;
}

const Vector2 Vector2::operator-() const {
    return {-x, -y};
}

bool Vector2::operator==(const Vector2& v) const {
    return x == v.x && y == v.y;
}

bool Vector2::operator!=(const Vector2& v) const {
    return !(*this == v);
}

const Vector3 Vector3::operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
}

const Vector3 Vector3::operator/(const double a) const {
    return Vector3(x / a, y / a, z / a);
}


double dot2(const Vector2 a, const Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

double dot3(const Vector3 a, const Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double cross2(const Vector2 a, const Vector2 b) {
    return a.x * b.y - a.y * b.x;
}

Vector3 cross3(const Vector3 a, const Vector3 b) {
    return Vector3{a.y * b.z - a.z * b.y,
                   a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x};
}

Vector2 triple_product(const Vector2 a, const Vector2 b, const Vector2 c) {
    return b * dot2(a, c) - c * dot2(a, b);
}

Vector2 proj2(const Vector2 A, const Vector2 B, const Vector2 v) {
    return (v * dot2(B - A, v) / (v.x * v.x + v.y * v.y));
}
