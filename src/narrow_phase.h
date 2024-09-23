#ifndef NARROW_PHASE_H
#define NARROW_PHASE_H

#include <vector>
#include <array>
#include "vector2.h"

class RigidBody;

struct Manifold {
    bool intersecting = false;
    Vector2 normal;
    double depth = 0;
    std::vector<Vector2> contact_points;
    unsigned count = 0;
};

struct ClosestPoints {
    Vector2 closest_a;
    Vector2 closest_b;
};

struct ProximityInfo {
    double distance = 0;
    ClosestPoints points;
};

/**
 * @brief Computes the support point of an object in a given direction
 * @return 
 */
Vector2 support(RigidBody* body, Vector2 d);

/**
 * @brief Computes the support point of an object in a given direction,
 * skipping the point given as parameter. In other words, if the latter
 * is a point laying on the boundary of the object, it is not taken
 * into account during the calculation of the support.
 * @param skip_me Point to ignore during the support calculation
 * @return 
 */
Vector2 support(RigidBody* body, Vector2 d, Vector2 skip_me);

Manifold detect_collision(RigidBody* a, RigidBody* b, double& gjk_time, double& epa_time);

/**
 * @brief Performs a proximity query: computes the euclidian distance between two convex shapes a and b, as well as their closest points from each other.
 * @param a Convex shape A
 * @param b Convex shape B
 * @return The proximity info, containing the distance and the closest points.
 */
ProximityInfo proximity_query(RigidBody* a, RigidBody* b);


#endif /* NARROW_PHASE_H */