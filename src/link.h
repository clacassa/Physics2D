#ifndef LINK_H
#define LINK_H

#include <SDL_render.h>
#include "vector2.h"
#include "utils.h"

class RigidBody;

class Spring {
public:
    enum DampingType { UNDAMPED = 0, UNDERDAMPED, CRIT_DAMPED, OVERDAMPED };

    Spring(RigidBody* A_, RigidBody* B_, double length, float stiffness, DampingType damping);
    void apply(const double dt);
    void draw(SDL_Renderer* renderer);
    double energy() const;

    // const ScrollingBuffer& get_phase_data() const { return phase_portrait; }
    inline float get_stiffness() const { return k; }
    inline double get_x_eq() const { return x_eq; }
    inline const Vector2& get_system_state() const { return system_state; }
    inline const Vector2 get_axis() const { return axis; }
    const Vector2 get_anchor() const;

private:
    RigidBody* A;
    RigidBody* B;
    Vector2 axis;
    double l0; // rest length
    float k; // stiffness
    double critical_damping; // c_crit = 2m sqrt(k / m)
    double actual_damping; // damping coefficient lambda
    Vector2 equilibrium_pos; // Equilibrium point in world space
    Vector2 system_state; // {x, x_dot}
    double x_eq;
    double theta;
    double theta_dot;

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
