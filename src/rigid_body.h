#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <SDL_render.h>
#include <cstddef>
#include <deque>
#include <string>
#include "shape.h"
#include "vector2.h"

enum BodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

constexpr double steel_density(7930 * 0.1); // kg / m^3 -> We are in 2D so factor 0.01 (1cm thick)
constexpr double steel_restitution(0.78 * 0.75); // Restitution is reduced for tuning behavior
constexpr double steel_static_friction(0.78 * 0.75);
constexpr double steel_dynamic_friction(0.42 * 0.75);

struct Friction {
    double f_static;
    double f_dynamic;
};

const Friction steel_friction = {steel_static_friction, steel_dynamic_friction};

struct RigidBodyDef {
    Vector2 position;
    Vector2 velocity;
    double rotation = 0;
    double angular_velocity = 0;
    double density = steel_density;
    double restitution = steel_restitution;
    Friction friction = steel_friction;
    BodyType type = DYNAMIC;
    bool enabled = true;
};

class RigidBody {
public:
    RigidBody(const RigidBodyDef& def, const Shape& shape, const size_t id);
    RigidBody(const RigidBodyDef& def, Shape* shape, const size_t id);
    RigidBody(const Shape& shape, const size_t id);
    RigidBody(Shape* shape, const size_t id);
    virtual ~RigidBody();

    void step(double dt);
    void subject_to_force(const Vector2 force, const Vector2 point);
    void subject_to_torque(const double torque);
    void reset_forces();
    void move(const Vector2 delta_p, bool update_AABB = true);
    void rotate(const double d_theta, bool update_AABB = true);
    void linear_impulse(const Vector2 impulse);
    void angular_impulse(const double impulse);
    void set_linear_vel(const Vector2 vel);
    void set_angular_vel(const double omega);
    void set_type(const BodyType type);
    
    double energy(double gravity) const;
    double k_energy() const;
    double p_energy(double gravity) const;

    void draw(SDL_Renderer* renderer);
    void draw_trail(SDL_Renderer* renderer, bool update_trace);
    void draw_bounding_box(SDL_Renderer* renderer);
    void draw_com(SDL_Renderer* renderer);
    void draw_forces(SDL_Renderer* renderer) const;
    void colorize(const SDL_Color color);
    void reset_color();

    std::string dump(double gravity) const;

    inline Vector2 get_a() const { return m_acc; }
    inline Vector2 get_v() const { return m_vel; }
    inline Vector2 get_p() const { return m_pos; }
    inline Vector2 get_f() const { return m_force; }
    inline double get_alpha() const { return m_alpha; }
    inline double get_omega() const { return m_omega; }
    inline double get_theta() const { return m_theta; }
    inline double get_mass() const { return m_mass; }
    inline double get_inv_m() const { return m_inv_mass; }
    inline double get_I() const { return m_inertia; }
    inline double get_inv_I() const { return m_inv_inertia; }
    inline double get_cor() const { return m_restitution; }
    inline Friction get_friction() const { return m_friction; }
    inline BodyType get_type() const { return m_type; }
    inline bool is_static() const { return m_type == STATIC; }
    inline bool is_dynamic() const { return m_type == DYNAMIC; }
    inline bool is_enabled() const { return m_enabled; }
    inline Shape* get_shape() const { return m_shape; }
    inline ShapeType get_shape_type() const { return m_shape->get_type(); }
    inline unsigned get_id() const { return m_id; }
    inline auto get_pos_curve() const { return trail; }

    void handle_wall_collisions();

protected:
    // linear, x y axis
    Vector2 m_acc;
    Vector2 m_vel;
    Vector2 m_pos;
    Vector2 m_force;
    // angular, z axis
    double m_alpha;
    double m_omega;
    double m_theta;
    double m_torque;

    double m_mass;
    double m_inv_mass;
    double m_inertia;
    double m_inv_inertia;
    double m_density;

    /*
     * coefficient of restitution (COR, e)   e = 0.69 for glass, 0.78 for stainless steel
     * https://en.wikipedia.org/wiki/Coefficient_of_restitution
     */
    double m_restitution;
    Friction m_friction;

    BodyType m_type;
    bool m_enabled;

    Shape* m_shape;

    size_t max_trail_length = 2e3;
    std::deque<Vector2> trail;

    SDL_Color m_color;

    size_t m_id;
};

#endif /* RIGID_BODY_H */
