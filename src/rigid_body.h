#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <SDL_render.h>
#include <cmath>
#include <deque>
#include <vector>
#include <string>
#include "vector2.h"

typedef std::vector<Vector2> Vertices;

enum BodyType { STATIC, KINEMATIC, DYNAMIC };

struct AABB {
    Vector2 min; // Bottom left corner
    Vector2 max; // Top right corner
};

constexpr double stl_steel_density(7930); // kg / m^3
constexpr double stl_steel_restitution(0.78 * 0.75); // Restitution is reduced for tuning behavior

class RigidBody {
public:
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_,
        Vertices verticies);
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_);
    virtual ~RigidBody();

    void step(double dt);
    void subject_to_force(const Vector2 force);
    void subject_to_torque(const Vector2 force);
    void reset_forces();
    void move(const Vector2 delta_p, bool update_AABB = true);
    void rotate(const double d_theta, bool update_AABB = true);
    void linear_impulse(const Vector2 impulse);
    void angular_impulse(const double impulse);
    
    double energy(bool gravity_enabled) const;
    double k_energy() const;
    double p_energy() const;

    virtual void draw(SDL_Renderer* renderer) const = 0;
    void draw_forces(SDL_Renderer* renderer) const;
    void draw_trace(SDL_Renderer* renderer, bool update_trace);
    void colorize(const SDL_Color color);
    void reset_color();

    std::string dump(bool gravity_enabled) const;

    inline Vector2 get_a() const { return a; }
    inline Vector2 get_v() const { return v; }
    inline Vector2 get_p() const { return p; }
    inline Vector2 get_f() const { return f; }
    inline double get_a_theta() const { return a_theta; }
    inline double get_theta() const { return theta; }
    inline double get_omega() const { return omega; }
    inline double get_mass() const { return m; }
    inline double get_inv_m() const { return inv_m; }
    inline double get_I() const { return I; }
    inline double get_inv_I() const { return inv_I; }
    inline double get_cor() const { return e; }
    inline BodyType get_type() const { return m_type; }
    inline bool is_static() const { return m_type == STATIC; }
    inline bool is_dynamic() const { return m_type == DYNAMIC; }
    inline bool is_enabled() const { return enabled; }
    inline std::string get_friction() const { return static_friction ? "STATIC" : "DYNAMIC"; }

    inline bool has_vertices() const { return !m_vertices.empty(); }
    inline virtual std::vector<Vector2> get_vertices() const { return m_vertices; }
    inline virtual double get_radius() const { return 0.0; }
    inline AABB get_AABB() const { return m_aabb; }

    inline void set_test(const size_t test) { id = test; }
    inline void set_friction_debug(const bool friction) { static_friction = friction; }

    virtual void handle_wall_collisions() = 0;
    virtual void update_bounding_box() = 0;
    virtual bool contains_point(const Vector2 point) const = 0;

protected:
    // linear, x y axis
#ifdef VERLET
    Vector2 p_old;
#endif
    Vector2 a;
    Vector2 v;
    Vector2 p;
    Vector2 f;
    // angular, z axis
    double a_theta;
    double omega;
    double theta;
    double torque;

    double m;
    double inv_m;
    double I;
    double inv_I;

    /*
     * coefficient of restitution (COR, e)   e = 0.69 for glass, 0.78 for stainless steel
     * https://en.wikipedia.org/wiki/Coefficient_of_restitution
     */
    double e;

    BodyType m_type;
    bool enabled;

    Vertices m_vertices;
    AABB m_aabb;

    size_t max_track_length;
    std::deque<Vector2> track;
    SDL_Color color;

    size_t id;
    bool static_friction;
};


class Ball : public RigidBody {
public:
    Ball(Vector2 vel, Vector2 pos, double m, double r_, BodyType type = DYNAMIC, bool enabled = true);
    virtual ~Ball() {}

    // A ball doesn't have any verticies
    Vertices get_vertices() const override;
    double get_radius() const override;
    void draw(SDL_Renderer* renderer) const override;
    void handle_wall_collisions() override;
    void update_bounding_box() override;
    bool contains_point(const Vector2 point) const override;

private:
    double r;
};

class Rectangle : public RigidBody {
public:
    Rectangle(Vector2 vel, Vector2 pos, double m, double w_, double h_, Vertices verticies, 
            BodyType type = DYNAMIC, bool enabled = true);
    virtual ~Rectangle() {}

    void draw(SDL_Renderer* renderer) const override;
    void handle_wall_collisions() override;
    void update_bounding_box() override;
    bool contains_point(const Vector2 point) const override;

private:
    double w;
    double h;
};

class Triangle : public RigidBody { 
// public:
//     Triangle(Vector2 vel, Vector2 pos, double m, Vertices vertices, bool movable = true);
//     virtual ~Triangle() {}

//     void draw(SDL_Renderer* renderer) const override;
//     void handle_wall_collisions() override;
//     void update_bounding_box() override;
//     bool contains_point(const Vector2 point) const override;
};

struct Frame {
    Frame(Vector2 center_, double w, double h) : center(center_), width(w), height(h) {}

    void draw(SDL_Renderer* renderer);

    Vector2 center;
    double width;
    double height;
};

#endif /* RIGID_BODY_H */

/*
Maybe implement a composition method for example Movable, NotMovable classes
that inherit from RigidBody base class, then the objects can derive from Movable
or NotMovable

Drawback: RigidBody must be an abstract class so every action outside the transition unit
must be done via methods and there will be no getter/setter methods
*/
