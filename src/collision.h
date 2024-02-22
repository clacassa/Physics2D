#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <SDL.h>
#include "rigid_body.h" 


constexpr unsigned GJK_max_iterations(1e4);
constexpr unsigned EPA_max_iterations(1e4);
constexpr double EPA_epsilon(1e-5);

struct CollisionInfo {
    bool intersecting;
    Vector2 normal;
    Vector2 contact_point;
    std::vector<Vector2> contact_points;
    double depth;

    CollisionInfo() : intersecting(false), depth(0) {}
};

void solve_collision(RigidBody* a, RigidBody* b, CollisionInfo collision);
void solve_wall_collision(RigidBody* body, CollisionInfo collision);

Vector2 support(RigidBody* body, Vector2 d);

/////// BROAD PHASE /////////////
class SweepAndPrune {
public:
    typedef std::array<RigidBody*, 2> BodyPair;

    SweepAndPrune() : m_var_x(0), m_var_y(0) {}
    virtual ~SweepAndPrune() { m_list.clear(); }
    void choose_axis(std::vector<RigidBody*> list);
    std::vector<BodyPair> process(std::vector<RigidBody*>& list);
    void sort_ascending_x(std::vector<RigidBody*>& list);
    void sort_ascending_y(std::vector<RigidBody*>& list);

private:
    std::vector<RigidBody*> m_list;
    double m_var_x;
    double m_var_y;
};

bool AABB_overlap(AABB a, AABB b);

/////// NARROW PHASE ///////////
CollisionInfo detect_collision(RigidBody* a, RigidBody* b, double& time_1, double& time_2);
// SAT
bool intersect_circle_circle(RigidBody* a, RigidBody* b, CollisionInfo& result);
bool intersect_circle_polygon(RigidBody* a, RigidBody* b, CollisionInfo& result);
bool intersect_polygon_polygon(RigidBody* a, RigidBody* b, CollisionInfo& result);

// A 2D simplex (point, segment or triangle)
typedef std::vector<Vector2> Simplex;
// Pairs of the points of A and B that created the Minkowski difference points
typedef std::vector<std::array<Vector2, 2>> SourcePoints;

/**
 * @brief GJK algorithm, detects the intersection between two convex shapes. GJK iteratively tries to find a simplex that contains the origin using features of the Minkowski difference of the two shapes.
 * @param s The initial simplex, empty
 * @param a Convex shape A
 * @param b Convex shape B
 * @return Whether the shapes intersect or not.
 */
bool intersect_GJK(Simplex& s, SourcePoints& shape_points, RigidBody* a, RigidBody* b);

/**
 * @brief Given a simplex, reduce it to its closest feature to the origin and find the direction to which it should be expanded in order to encompass the origin.
 * @param s The simplex from the previous iteration
 * @param direction The new direction towards the origin
 * @return Whether the newly created simplex contains the origin
 */
bool nearest_simplex(Simplex& s, Vector2& direction, SourcePoints& points);

struct Edge {
    double distance;
    Vector2 normal;
    size_t index;

    Edge() : distance(0), normal(0, 0), index(0) {}
};

/**
 * @brief Expanding Polytype Algorithm, finds the point in Minkowski difference the closest to the origin. EPA starts from the simplex given by GJK and iteratively expand it until one of its edges is close enough to the closest point to the origin on the Minkowski difference countour. It can then decuce the collision normal, the penetration depth and the contact points.
 * @param s Simplex given by GJK
 * @param a Convex shape A
 * @param b Convex shape B
 * @param shape_points The source points of A and B that were used to create the simplex
 * @param result The resulting information of the collision (normal, depth, contact points)
 */
void EPA(Simplex s, SourcePoints points, RigidBody* a, RigidBody* b, CollisionInfo& result);
/**
 * @brief Given a simplex, find its closest edge to the origin.
 * @param s Simplex containing the origin
 * @param clockwise Whether the simplex is CW or CCW oriented
 * @return The closest edge to the origin of the simplex.
 */
Edge closest_edge(Simplex s, const bool clockwise);

struct ClosestPoints {
    Vector2 closest_a;
    Vector2 closest_b;
};

ClosestPoints convex_combination(Simplex s, SourcePoints shape_points, size_t index);


#endif /* COLLISION_H */
