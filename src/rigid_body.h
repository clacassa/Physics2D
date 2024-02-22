#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <cmath>
#include <deque>
#include <vector>
#include <string>
#include "render.h"
#include "vector2.h"

#define PI 3.14159265

constexpr double g(9.81);

struct AABB {
    Vector2 min; // Bottom left corner
    Vector2 max; // Top right corner
};

typedef std::vector<Vector2> Vertices;


class RigidBody {
public:
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, bool movable_, bool enabled_,
        Vertices verticies);
    RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, bool movable_, bool enabled_);
    virtual ~RigidBody();

    void step(double dt);
    void subject_to_force(const Vector2 force);
    void subject_to_torque(const Vector2 force);
    void reset_forces();
    void move(const Vector2 delta_p, bool update_AABB = true);
    void rotate(const double angle, bool update_AABB = true);
    void velocity_impulse(const Vector2 impulse);
    void angular_impulse(const double impulse);
    
    double energy(bool gravity_enabled) const;
    double k_energy() const;
    double p_energy() const;

    virtual void draw(SDL_Renderer* renderer) const = 0;
    void draw_forces(SDL_Renderer* renderer) const;
    void draw_trace(SDL_Renderer* renderer);
    void colorize(const SDL_Color color);
    void reset_color();

    std::string dump(bool gravity_enabled) const;

    Vector2 get_a() const { return a; }
    Vector2 get_v() const { return v; }
    Vector2 get_p() const { return p; }
    Vector2 get_f() const { return f; }
    double get_omega() const { return omega; }
    double get_mass() const { return m; }
    double get_inv_m() const { return inv_m; }
    double get_I() const { return I; }
    double get_inv_I() const { return inv_I; }
    double get_cor() const { return e; }
    bool is_movable() const { return movable; }
    bool is_enabled() const { return enabled; }
    std::string get_friction() const { return static_friction ? "STATIC" : "DYNAMIC"; }

    bool has_vertices() const { return !m_vertices.empty(); }
    virtual std::vector<Vector2> get_vertices() const { return m_vertices; }
    virtual double get_radius() const { return 0.0; }
    AABB get_AABB() const { return m_aabb; }

    virtual void handle_wall_collisions() = 0;
    virtual void update_bounding_box() = 0;
    virtual bool contains_point(const Vector2 point) const = 0;

    void set_test(const Vector2 test) { m_t = test; }
    void set_test(const size_t test) { id = test; }
    void set_friction_debug(const bool friction) { static_friction = friction; }

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

    bool movable;
    bool enabled;

    Vertices m_vertices;
    AABB m_aabb;

    size_t max_track_length;
    std::deque<Vector2> track;
    SDL_Color color;    

    Vector2 m_t;
    size_t id;
    bool static_friction;
};


class Ball : public RigidBody {
public:
    Ball(Vector2 vel, Vector2 pos, double m, double r_, bool movable = true, bool enabled = true);
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
            bool movable = true, bool enabled = true);
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