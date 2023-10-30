#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <cmath>
#include <deque>
#include "render.h"

#define PI 3.14159265

constexpr double g(9.81);

struct Vector2 {
    double x;
    double y;

    Vector2(double x_, double y_) : x(x_), y(y_) {}
    Vector2() : x(0.0), y(0.0) {}

    double norm() const;
    Vector2 normalized() const;

    const Vector2 operator+(const Vector2& v) const;
    Vector2& operator+=(const Vector2& v);
    const Vector2 operator-(const Vector2& v) const;
    Vector2& operator-=(const Vector2& v);
    const Vector2 operator*(const double a) const;
    const Vector2 operator/(const double a) const;
};

double dot2(const Vector2 a, const Vector2 b);
// a x b 
double cross2(const Vector2 a, const Vector2 b);


struct RigidBody {
    enum class Shape { Disc, Polygon };
    // RigidBody(double v_x, double v_y, double x, double y, double m_, double I_, Shape);
    RigidBody(double v_x, double v_y, double x, double y, double m_, double r_);
    virtual ~RigidBody();

    double energy() const;
    double k_energy() const;
    double p_energy() const;

    void draw(SDL_Renderer* renderer);
    void draw_forces(SDL_Renderer* renderer) const;

    // linear, x y axis
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
    double r;
    double I;

    size_t max_track_length;
    std::deque<Vector2> track;
    Shape m_shape;
};

struct Ball : public RigidBody {
    Ball(double v_x, double v_y, double x, double y, double m_, double r_);
    virtual ~Ball() {}

    double r;
};

struct Polygon : public RigidBody {
    Polygon(double v_x, double v_y, double x, double y, double m_, double I_);
    virtual ~Polygon() {}


};

struct Rectangle : public Polygon {
    Rectangle(double v_x, double v_y, double x, double y, double m_, double w_, double h_);
    virtual ~Rectangle() {}
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