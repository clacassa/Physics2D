#ifndef NARROW_PHASE_H
#define NARROW_PHASE_H

#include <array>
#include "vector2.h"

struct Timer;
class Shape;

struct Manifold {
    bool intersecting = false;
    Vector2 normal;
    double depth = 0;
    std::array<Vector2, 2> contact_points;
    unsigned count = 0;
};

struct ClosestPoints {
    Vector2 closest_a;
    Vector2 closest_b;
};

struct DistanceInfo {
    double distance = 0;
    ClosestPoints points;
};

/**
 * @brief Computes the support point of a convex shape following a given direction
 * @return 
 */
Vector2 support(const Shape* shape, const Vector2 d);

// SAT
Manifold collide_circle_circle(Shape* a, Shape* b);
Manifold collide_circle_polygon(Shape* a, Shape* b);
Manifold collide_polygon_polygon(Shape* a, Shape* b);

/**
 * @brief Determines if two convex shapes are colliding, and computes the contact manifold.
 * Uses GJK for colliion detection, EPA for penetration vector,
 * and clipping for contact point(s) calculation.
 * @return The contact manifold, containing all the information needed to solve the collision.
 */
Manifold collide_convex(Shape* a, Shape* b, Timer& gjk, Timer& epa, Timer& clip);

/**
 * @brief Performs a proximity query: computes the euclidian distance between two convex shapes a and b, as well as their closest points from each other.
 * @param a Convex shape A
 * @param b Convex shape B
 * @return The proximity info, containing the distance and the closest points.
 */
DistanceInfo ditance_convex(const Shape* a, const Shape* b);


#endif /* NARROW_PHASE_H */
