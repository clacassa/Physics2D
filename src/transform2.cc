#include <cassert>
#include <array>
#include <cmath>
#include "transform2.h"

namespace {
    struct Mat22 {
        std::array<Vector2, 2> mat;

        Mat22(const Vector2 row1, const Vector2 row2) {
            mat[0] = row1;
            mat[1] = row2;
        }

        static Mat22 rot(const double theta) {
            return Mat22(Vector2(cos(theta), -sin(theta)), Vector2(sin(theta), cos(theta)));
        }

        Vector2 mul(const Vector2 b) {
            return Vector2(dot2(mat[0], b), dot2(mat[1], b));
        }
    };
}

Vector2 transform2(const Vector2 v0, const Vector2 t, const double q, const Vector2 axis) {
    return Mat22::rot(q).mul(v0 - axis) + axis + t;
}
