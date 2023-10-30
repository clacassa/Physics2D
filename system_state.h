#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "render.h"
#include "rigid_body.h"
#include "ODE_solver.h"
#include "collision.h"
#include "link.h"


class SystemState {
public:
    const double air_viscosity = 1.48e-5;

    SystemState();
    virtual ~SystemState();

    void initialize(EulerSolver* solver);
    void process(double dt, int steps, bool perft = false);
    void apply_forces();
    void toggle_gravity();

    void add_body(Vector2 pos, Vector2 vel = {0, 0}, double mass = 1.0, double radius = 0.1);
    void add_frame(Vector2 pos);
    void add_link(Vector2 p1, Vector2 p2);

    void render(SDL_Renderer* renderer);
    
    std::string dump_object_data() const;
    void focus_next();
    void focus_prev();

    size_t get_body_count() const { return m_bodies.size(); };

private:
    bool gravity;
    std::vector<RigidBody*> m_bodies;
    std::vector<Frame*> m_frames;
    std::array<Vector2, 3> m_force_fields;
    std::vector<Constraint*> m_constraints;
    std::vector<FixedLink*> m_fixed_links;

    EulerSolver* m_solver;
    CollisionGrid m_grid;

    double m_dt;
    double step_time;

    size_t focus;
};

#endif /* SYSTEM_STATE_H */
