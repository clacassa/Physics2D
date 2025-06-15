// 2D vectorial transformations using homogeneous matrices

#ifndef TRANSFORM2_H
#define TRANSFORM2_H

#include "vector2.h"

/**
 * @brief Performs a 2D transform of vector v0 translated by t and rotated by q. A point of rotation different than the origin can be specified.
 * @param v0 The vector to transform
 * @param t The translation vector
 * @param q The rotation angle
 * @param axis The point of rotation (origin by default)
 * @return 
 */
Vector2 transform2(const Vector2 v0, const Vector2 t, const double q, const Vector2 axis = vector2_zero);

#endif /* TRANSFORM2_H */