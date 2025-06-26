#ifndef WORLD_H
#define WORLD_H

#include <cstddef>
#include <vector>
#include <array>
#include <string>
#include "broad_phase.h" // SweepAndPrune
#include "link.h"        // Spring::DampingType
#include "rigid_body.h"

struct Settings;
struct DistanceInfo;
struct Manifold;
class RigidBody;
class Shape;

class World {
public:
    World();
    virtual ~World();

    void step(double dt, int steps, Settings& settings, bool perft = false);
    void render(SDL_Renderer* renderer, bool running, Settings& settings);

    RigidBody* create_body(const RigidBodyDef& body_def, const Shape& shape);
    void add_spring(Vector2 p1, Vector2 p2, Spring::DampingType damping, float stiffness);

    void destroy_body(RigidBody* body);
    void destroy_all();
    
    std::string dump_profile() const;
    std::string dump_selected_body() const;
    double total_energy() const;

    bool focus_next();
    bool focus_prev();
    bool focus_on_position(Vector2 p);
    bool focus_from_id(const size_t id);

    inline size_t get_focus() const { return focus; }
    RigidBody* get_focused_body() const;
    RigidBody* get_body_from_id(const size_t id) const;

    inline unsigned get_body_count() const { return body_count; }
    inline void toggle_gravity() { gravity_enabled = !gravity_enabled; }
    inline void enable_gravity() { gravity_enabled = 1; }
    inline void disable_gravity() { gravity_enabled = 0; }
    inline void enable_walls() { walls_enabled = 1; }
    inline void disable_walls() { walls_enabled = 0; }
    
private:
    // Struct to store performance metrics
    struct Profile {
        double step;
        double ode;
        double collisions;
        double broad_phase;
        double pairs;
        double AABBs;
        double narrow_phase;
        double gjk_collide;
        double epa;
        double clip;
        double response_phase;
        double walls;

        void reset();
    };

    bool gravity_enabled;
    bool walls_enabled;
    bool air_friction_enabled;

    std::vector<RigidBody*> m_bodies;
    unsigned body_count;
    unsigned focus;

    std::vector<Manifold*> m_contacts;
    std::vector<DistanceInfo*> m_proxys;

    std::vector<Spring*> m_springs;
    std::array<Vector2, 3> m_force_fields;
    // std::vector<Constraint*> m_constraints;
    SweepAndPrune m_sap;
    Profile m_profile;
    
    void apply_forces();
    Manifold collide(RigidBody* body_a, RigidBody* body_b);

    void destroy_contacts();
    void destroy_proxys();
};

#endif /* WORLD_H */
