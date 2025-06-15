#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <SDL_render.h>
#include <cmath>
#include <deque>
#include <vector>
#include <string>
#include "shape.h"
#include "vector2.h"

enum BodyType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

constexpr double steel_density(7930); // kg / m^3
constexpr double steel_restitution(0.78 * 0.75); // Restitution is reduced for tuning behavior
constexpr double steel_static_friction(0.78);
constexpr double steel_dynamic_friction(0.42);

struct RigidBodyDef {
    Vector2 position;
    Vector2 velocity;
    double rotation = 0;
    double angular_velocity = 0;
    double density = steel_density;
    double restitution = steel_restitution;
    BodyType type = DYNAMIC;
    bool enabled = true;
};

class RigidBody {
public:
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_,
        Vertices verticies);
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_);
    RigidBody(const RigidBodyDef& def, Shape* shape);
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
    
    double energy(bool gravity_enabled) const;
    double k_energy() const;
    double p_energy() const;

    void draw(SDL_Renderer* renderer);
    void draw_trace(SDL_Renderer* renderer, bool update_trace);
    void draw_bounding_box(SDL_Renderer* renderer);
    void draw_forces(SDL_Renderer* renderer) const;
    void colorize(const SDL_Color color);
    void reset_color();

    std::string dump(bool gravity_enabled) const;

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
    inline BodyType get_type() const { return m_type; }
    inline bool is_static() const { return m_type == STATIC; }
    inline bool is_dynamic() const { return m_type == DYNAMIC; }
    inline bool is_enabled() const { return m_enabled; }
    inline Shape* get_shape() const { return m_shape; }
    inline ShapeType get_shape_type() const { return m_shape->get_type(); }

    // inline bool has_vertices() const { return !m_vertices.empty(); }
    // inline virtual std::vector<Vector2> get_vertices() const { return m_vertices; }
    // inline virtual double get_radius() const { return 0.0; }
    // inline AABB get_AABB() const { return m_shape->get_aabb(); }

    void handle_wall_collisions();
    // virtual void update_bounding_box() = 0;
    // virtual bool contains_point(const Vector2 point) const = 0;

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

    BodyType m_type;
    bool m_enabled;

    // Vertices m_vertices;
    // AABB m_aabb;
    Shape* m_shape;

    size_t max_track_length = 1e3;
    std::deque<Vector2> track;
    SDL_Color m_color;

    size_t m_id;
};


// class Ball : public RigidBody {
// public:
//     Ball(Vector2 vel, Vector2 pos, double m, double r_, BodyType type = DYNAMIC, bool enabled = true);
//     virtual ~Ball() {}
//
//     // A ball doesn't have any verticies
//     // Vertices get_vertices() const override;
//     // double get_radius() const override;
//     void draw(SDL_Renderer* renderer) override;
//     void handle_wall_collisions() override;
//     // void update_bounding_box() override;
//     // bool contains_point(const Vector2 point) const override;
//
// // private:
// //     double r;
// };

// class Rectangle : public RigidBody {
// public:
//     Rectangle(Vector2 vel, Vector2 pos, double m, double w_, double h_, Vertices verticies,
//             BodyType type = DYNAMIC, bool enabled = true);
//     virtual ~Rectangle() {}
//
//     void draw(SDL_Renderer* renderer) override;
//     void handle_wall_collisions() override;
//     // void update_bounding_box() override;
//     // bool contains_point(const Vector2 point) const override;
//
// // private:
// //     double w;
// //     double h;
// };

// class Triangle : public RigidBody { 
// public:
//     Triangle(Vector2 vel, Vector2 pos, double m, Vertices vertices, bool movable = true);
//     virtual ~Triangle() {}

//     void draw(SDL_Renderer* renderer) const override;
//     void handle_wall_collisions() override;
//     void update_bounding_box() override;
//     bool contains_point(const Vector2 point) const override;
// };

#endif /* RIGID_BODY_H */

/*
Maybe implement a composition method for example Movable, NotMovable classes
that inherit from RigidBody base class, then the objects can derive from Movable
or NotMovable

Drawback: RigidBody must be an abstract class so every action outside the transition unit
must be done via methods and there will be no getter/setter methods
*/
