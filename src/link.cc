#include "link.h"
#include "render.h"
#include <iostream>


Spring::Spring(RigidBody* A_, RigidBody* B_, double stiffness, double length, DampingType damping)
:   A(A_),
    B(B_),
    k(stiffness),
    l_0(length)
{
    const double m(A->get_mass() * B->get_mass() / (A->get_mass() + B->get_mass()));
    critical_damping = 2 * sqrt(k * m);

    switch (damping) {
        case UNDAMPED: 
            actual_damping = 0;
            break;
        case UNDERDAMPED: 
            actual_damping = 0.05 * critical_damping;
            break;
        case CRIT_DAMPED:
            actual_damping = critical_damping;
            break;
        case OVERDAMPED:
            actual_damping = 10 * critical_damping;
            break;
    }
}

void Spring::apply() {
    Vector2 axis(A->get_p() - B->get_p());
    double l(axis.norm());

    const Vector2 n(axis.normalized());
    // Calculate equilibrum position
    if (!A->is_movable() || !A->is_enabled())
        equilibrum_pos = A->get_p() - n * (l_0 + dot2(B->get_f(), n) / k);
    else if (!B->is_movable() || !B->is_enabled())
        equilibrum_pos = B->get_p() + n * (l_0 + dot2(A->get_f(), n) / k);

    // Calculate and apply spring forces on each body
    const double callback(k * (l - l_0));
    const double damping(actual_damping * dot2((A->get_v() - B->get_v()), n));
    if (!B->is_movable())
        A->subject_to_force(-n * (callback + damping));
    else if (!A->is_movable())
        B->subject_to_force(n * (callback + damping));
    else {
        A->subject_to_force(-n * (callback + damping) * 0.5);
        B->subject_to_force(n * (callback + damping) * 0.5);
    }

    position_curve.push_back(l - l_0);
}

void Spring::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);
    render_line(renderer, B->get_p().x, B->get_p().y, A->get_p().x, A->get_p().y);
    // if (A->is_movable())
    //     A->draw_trace(renderer);
    // if (B->is_movable())
    //     B->draw_trace(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    render_line(renderer, 0, 0, equilibrum_pos.x, equilibrum_pos.y);
}

double Spring::energy() const {
    const double x(Vector2(B->get_p() - A->get_p()).norm() - l_0);
    return 0.5 * k * x * x;
}


// Spring::Spring(RigidBody* A, RigidBody* B, double stiffness, double length)
// :   a(A),
//     b(B),
//     k(stiffness),
//     l_0(length)
// {}

// void Spring::apply() {
//     Vector2 axis(a->get_p() - b->get_p());
//     double l(axis.norm());
//     Vector2 n(axis.normalized());
//     Vector2 callback_force(n * k * (l - l_0));

//     if (!a->is_movable())
//         b->subject_to_force(callback_force);
//     else if (!b->is_movable())
//         a->subject_to_force(-callback_force);
//     else {
//         a->subject_to_force(-callback_force * 0.5);
//         b->subject_to_force(callback_force * 0.5);
//     }
// }

// void Spring::draw(SDL_Renderer* renderer) {
//     SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
//     render_line(renderer, a->get_p().x, a->get_p().y, b->get_p().x, b->get_p().y);
//     // draw_center_of_mass(renderer);
//     b->draw_trace(renderer);
// }

// double Spring::energy() const {
//     const double x(Vector2(a->get_p() - b->get_p()).norm() - l_0);
//     return 0.5 * k * x * x;
// }

// void Spring::draw_center_of_mass(SDL_Renderer* renderer) {
//     if (track.size() == 100) {
//         track.pop_front();
//     }
//     double m_a(a->get_mass());
//     double m_b(b->get_mass());
//     Vector2 p_a(a->get_p());
//     Vector2 p_b(b->get_p());
//     Vector2 c_m((p_a * m_a + p_b * m_b) / (m_a + m_b));
//     if (track.size() < 100)
//         track.push_back(c_m);

//     size_t i(0);
//     for (auto point : track) {
//         int alpha(255 / track.size() * i);
//         SDL_SetRenderDrawColor(renderer, 255, 0, 0, alpha);
//         render_filled_circle(renderer, point.x, point.y, 1 / (double)RENDER_SCALE);
//         ++i;
//     }
// }


#ifdef VERLET
void Link::apply() {

    Vector2 axis(a->get_p() - b->get_p());
    const Vector2 n(axis.normalized());
    const double delta(axis.norm() - distance);
    // a->move(n * 0.5 * delta);
    // b->move(n * -0.5 * delta);
    double M(a->get_mass() * b->get_mass() / (a->get_mass() + b->get_mass()));
    a->subject_to_force(n * -(/*dot2(a->get_a() * a->get_mass(), axis)*/ + delta)  * M * M);
    b->subject_to_force(n * (/*dot2(b->get_a() * b->get_mass(), axis)*/ + delta)  * M * M);
}

void Link::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    render_line(renderer, a->get_p().x, a->get_p().y, b->get_p().x, b->get_p().y);
}

FixedLink::FixedLink(RigidBody* A, Frame* F, double dist)
:   a(A),
    f(F),
    distance(dist),
    pulsation(sqrt(g / distance)),
    m_dt(0.0)
{
    Vector2 axis(f->center - a->get_p());
    if (axis.norm() != distance) {
        std::cout << "merde\n";
        exit(1);
    }
    theta_0 = -PI / 2 + atan2(axis.y, axis.x);
    theta = theta_0;
    // std::cout << theta_0 << " " << pulsation << " " << distance << "\n";
}

void FixedLink::apply(/*double dt*/) {
    double l(distance);
    // std::cout << "old: " << m_dt << "\n";
    Vector2 r(f->center - a->get_p());
    Vector2 n(r.normalized());
    a->move(n * (r.norm() - l));

    // m_dt += dt;
    // double theta_new(theta_0 * cos(pulsation * m_dt));
    // Vector2 axis(f->center - body->get_p());
    // // Vector2 n(axis.normalized());
    // double theta_new(-PI / 2 + atan2(axis.y, axis.x));
    // // double omega((theta - theta_new) / dt);
    // theta = theta_new;
    // // double tension(body->m * l * omega * omega + body->m * g * cos(theta));
    // double tension(body->get_mass() * g * (3*cos(theta) - 2*cos(theta_0)));
    // Vector2 T(-tension * sin(theta), tension * cos(theta));
    // body->subject_to_force(T);
    // std::cout << axis.norm() << "\n";
    // std::cout << T.x << " " << T.y << "\n";
}

void FixedLink::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    render_line(renderer, a->get_p().x, a->get_p().y, f->center.x, f->center.y);
}
#endif /* VERLET */