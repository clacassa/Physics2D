#ifndef COLLISION_H
#define COLLISION_H

struct Manifold;
class RigidBody;

/*
* Impulse-based reaction model
* https://en.wikipedia.org/wiki/Collision_response
*/
void solve_collision(RigidBody* a, RigidBody* b, const Manifold& collision);
void solve_wall_collision(RigidBody* body, const Manifold& collision);

#endif /* COLLISION_H */
