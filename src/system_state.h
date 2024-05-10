#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <chrono>
#include "collision.h"  // SweepAndPrune
#include "link.h"       // Spring::DampingType
#include "rigid_body.h"

// Forward declarations
struct Settings;
class RigidBody;

class SystemState {
public:
    const double air_viscosity = 1.48e-5;
    const SDL_Color focus_color = {255, 0, 255, 255};

    SystemState();
    virtual ~SystemState();

    void process(double dt, int steps, Settings& settings, bool perft = false);
    void render(SDL_Renderer* renderer, bool running, Settings& settings);

    void toggle_gravity();

    void add_ball(Vector2 pos, double radius, BodyType type = DYNAMIC, bool enabled = true,
            Vector2 vel = {0, 0});
    void add_rectangle(Vector2 pos, double width, double height, BodyType type = DYNAMIC,
            bool enabled = true, Vector2 vel = {0, 0});
    void add_spring(Vector2 p1, Vector2 p2, Spring::DampingType damping, float stiffness);

    void destroy_all();
    
    std::string dump_metrics() const;
    std::string dump_selected_body() const;
    double total_energy() const;

    void focus_next();
    void focus_prev();
    void focus_on_position(Vector2 p);
    void move_focused_body(Vector2 delta_p);
    void rotate_focused_body(double angle);

    inline unsigned get_body_count() const { return body_count; }
    RigidBody* get_focused_body() const;

private:
    // Struct to store performance metrics
    struct PerfMetrics {
        double step_time;
        double ode_time;
        double collisions_time;
        double broad_phase;
        double narrow_phase;
        double gjk_time;
        double epa_time;
        double response_phase;

        void reset();
        void average(double steps);
    };

    bool gravity_enabled;
    bool air_friction_enabled;

    std::vector<RigidBody*> m_bodies;
    unsigned body_count;
    unsigned focus;

    std::vector<Manifold*> m_contacts;

    std::vector<Spring*> m_springs;
    std::array<Vector2, 3> m_force_fields;
    // std::vector<Constraint*> m_constraints;
    SweepAndPrune m_sap;
    PerfMetrics m_perf_metrics;
    
    void apply_forces();
};

#endif /* SYSTEM_STATE_H */
