#ifndef LINK_H
#define LINK_H

#include <SDL_render.h>
#include <deque>
#include <vector>
#include "vector2.h"

class RigidBody;

class Spring {
public:
    enum DampingType { UNDAMPED = 0, UNDERDAMPED, CRIT_DAMPED, OVERDAMPED };

    Spring(RigidBody* A_, RigidBody* B_, double length, float stiffness, DampingType damping);
    void apply();
    void draw(SDL_Renderer* renderer);
    double energy() const;

private:
    RigidBody* A;
    RigidBody* B;
    double l_0; // rest length
    float k; // stiffness
    double critical_damping; // c_crit = 2m sqrt(k / m)
    double actual_damping; // damping coefficient lambda
    Vector2 equilibrum_pos;
    std::vector<double> position_curve;

    void draw_coil(SDL_Renderer* renderer, Vector2 start, Vector2 direction, double height) const; 
};

// Obsolete
#ifdef VERLET
class Link : public Constraint {
public:
    Link(RigidBody* A, RigidBody* B, double dist) : a(A), b(B), distance(dist) {}
    void apply() override;
    void draw(SDL_Renderer* renderer) override;
    double energy() const override { /* stub */ return 0.0;}
    
private:
    RigidBody* a;
    RigidBody* b;
    double distance;
};

struct FixedLink : public Constraint {
    FixedLink(RigidBody* A, Frame* F, double dist);
    void apply(/*double dt*/) override;
    void draw(SDL_Renderer* renderer) override;
    double energy() const override { return 0.0; }

    RigidBody* a;
    Frame* f;
    double distance;
    double theta_0;
    double pulsation;
    double theta;
    double m_dt;
};
#endif /* VERLET */

#endif /* LINK_H */
