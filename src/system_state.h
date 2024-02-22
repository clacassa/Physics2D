#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "render.h"
#include "rigid_body.h"
#include "collision.h"
#include "link.h"


class SystemState {
public:
    const double air_viscosity = 1.48e-5;
    const SDL_Color focus_color = {255, 0, 255, 255};

    SystemState();
    virtual ~SystemState();

    void process(double dt, int steps, bool perft = false);
    void apply_forces();
    void toggle_gravity();

    void add_ball(Vector2 pos, double mass, double radius, bool movable = true, bool enabled = true,
            Vector2 vel = {0, 0});
    void add_rectangle(Vector2 pos, double mass, double width, double height, bool movable = true,
            bool enabled = true, Vector2 vel = {0, 0});
    void add_spring(Vector2 p1, Vector2 p2, Spring::DampingType damping, bool very_stiff);
    void move_focused_body(Vector2 delta_p);
    void rotate_focused_body(double angle);

    void render(SDL_Renderer* renderer);
    
    std::string dump_data() const;
    void focus_next();
    void focus_prev();
    void focus_on_position(Vector2 p);

    size_t get_body_count() const { return m_bodies.size(); };

    // Custom struct to store time analytics variables
    struct TimePerf {
        double step_time;
        double ode_time;
        double collisions_time;
        double broad_phase;
        double narrow_phase;
        double GJK;
        double EPA;
        double response_phase;

        void reset();
        void average(double steps);
    };

private:
    bool gravity_enabled;
    bool air_friction_enabled;

    std::vector<RigidBody*> m_bodies;
    unsigned body_count;
    unsigned focus;

    std::vector<Spring*> m_springs;
    std::array<Vector2, 3> m_force_fields;
    // std::vector<Constraint*> m_constraints;
    SweepAndPrune m_SAP;
    TimePerf m_Time_Perf;
};

#endif /* SYSTEM_STATE_H */
