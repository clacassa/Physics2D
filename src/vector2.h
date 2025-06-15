#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>

constexpr double PI(3.14159265);

double deg2rad(const double deg_angle);
double rad2deg(const double rad_angle);

struct Vector2 {
    double x;
    double y;

    Vector2(double x_, double y_) : x(x_), y(y_) {}
    Vector2() : x(0.0), y(0.0) {}

    double norm() const;
    Vector2 normal() const;
    Vector2 normalized() const;

    const Vector2 operator+(const Vector2& v) const;
    const Vector2 operator-(const Vector2& v) const;
    const Vector2 operator*(const double a) const;
    const Vector2 operator/(const double a) const;

    Vector2& operator+=(const Vector2& v);
    Vector2& operator-=(const Vector2& v);
    Vector2& operator*=(const double a);

    // Negate the vector
    const Vector2 operator-() const;
    
    // Comparators
    bool operator==(const Vector2& v) const;
    bool operator!=(const Vector2& v) const;
};

const Vector2 vector2_zero(0, 0);
const Vector2 vector2_x(1, 0);
const Vector2 vector2_y(0, 1);
const Vector2 vector2_xy(1, 1);
const Vector2 vector2_q1(cos(0.25 * PI), sin(0.25 * PI));
const Vector2 vector2_q2(cos(0.75 * PI), sin(0.75 * PI));
const Vector2 vector2_q3(-vector2_q1);
const Vector2 vector2_q4(-vector2_q2);

struct Vector3 {
    double x;
    double y;
    double z;

    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vector3() : x(0.0), y(0.0), z(0.0) {}

    const Vector3 operator+(const Vector3& v) const;
    const Vector3 operator/(const double a) const;
};

double dot2(const Vector2 a, const Vector2 b);
double dot3(const Vector3 a, const Vector3 b);
// a x b 
double cross2(const Vector2 a, const Vector2 b);
Vector3 cross3(const Vector3 a, const Vector3 b);
// Triple product a x (b x c)
Vector2 triple_product(const Vector2 a, const Vector2 b, const Vector2 c);
// Orthogonal projection of A on v with B a point on v
Vector2 proj2(const Vector2 A, const Vector2 B, const Vector2 v);

#endif /* VECTOR2_H */
