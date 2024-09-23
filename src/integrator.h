#ifndef INTEGRATOR_H
#define INTEGRATOR_H

class RigidBody;

void rk4_step(RigidBody* body, double dt);

#endif /* INTEGRATOR_H */