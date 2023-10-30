#include "link.h"
#include "render.h"
#include <iostream>

void Link::apply() {
    RigidBody& body_1(*a);
    RigidBody& body_2(*b);

    Vector2 axis(body_1.p - body_2.p);
    const Vector2 n(axis.normalized());
    const double delta(distance - axis.norm());
    body_1.p += n * 0.5 * delta;
    body_2.p -= n * 0.5 * delta;
}

void Link::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    render_line(renderer, a->p.x, a->p.y, b->p.x, b->p.y);
}

FixedLink::FixedLink(RigidBody* A, Frame* F, double dist)
:   a(A),
    f(F),
    distance(dist),
    pulsation(sqrt(g / distance)),
    m_dt(0.0)
{
    Vector2 axis(f->center - a->p);
    if (axis.norm() != distance) {
        std::cout << "merde\n";
        exit(1);
    }
    theta_0 = -PI / 2 + atan2(axis.y, axis.x);
    theta = theta_0;
    std::cout << theta_0 << " " << pulsation << " " << distance << "\n";
}

void FixedLink::apply(double dt) {
    RigidBody* body(a);
    double l(distance);
    // std::cout << "old: " << m_dt << "\n";
    Vector2 r(f->center - a->p);
    Vector2 n(r.normalized());
    if (r.norm() < l) {
        a->p += n * (r.norm() - l);
    }else if (r.norm() > l) {
        a->p += n * (r.norm() - l);
    }

    // m_dt += dt;
    // double theta_new(theta_0 * cos(pulsation * m_dt));
    Vector2 axis(f->center - body->p);
    // Vector2 n(axis.normalized());
    double theta_new(-PI / 2 + atan2(axis.y, axis.x));
    // double omega((theta - theta_new) / dt);
    theta = theta_new;
    // double tension(body->m * l * omega * omega + body->m * g * cos(theta));
    double tension(body->m * g * (3*cos(theta) - 2*cos(theta_0)));
    Vector2 T(-tension * sin(theta), tension * cos(theta));
    body->f += T;
    // std::cout << axis.norm() << "\n";
    // std::cout << T.x << " " << T.y << "\n";
}


FixedSpring::FixedSpring(RigidBody* A, Frame* F, double stiffness, double length)
:   a(A),
    f(F),
    k(stiffness),
    l_0(length)
{}

void FixedSpring::apply() {
    Vector2 axis(f->center - a->p);
    double l(axis.norm());

    Vector2 n(axis.normalized());
    Vector2 callback_force(n * k * (l - l_0));
    a->f += callback_force;
}

void FixedSpring::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    render_line(renderer, f->center.x, f->center.y, a->p.x, a->p.y);
}

double FixedSpring::energy() const {
    const double x(Vector2(f->center - a->p).norm() - l_0);
    return 0.5 * k * x * x;
}


Spring::Spring(RigidBody* A, RigidBody* B, double stiffness, double length)
:   a(A),
    b(B),
    k(stiffness),
    l_0(length)
{}

void Spring::apply() {
    Vector2 axis(a->p - b->p);
    double l(axis.norm());

    Vector2 n(axis.normalized());
    Vector2 callback_force(n * k * (l - l_0));
    a->f -= callback_force * 0.5;
    b->f += callback_force * 0.5;
}

void Spring::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    render_line(renderer, a->p.x, a->p.y, b->p.x, b->p.y);
}

double Spring::energy() const {
    const double x(Vector2(a->p - b->p).norm() - l_0);
    return 0.5 * k * x * x;
}
