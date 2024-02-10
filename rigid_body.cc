#include <iostream>
#include <array>
#include "rigid_body.h"
#include "collision.h"
#include "config.h"


RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, bool movable_, 
        Vertices vertices)
:   
#ifdef Verlet
    p_old(pos),
#endif
    a({0, 0}),
    v(vel),
    p(pos),
    f({0, 0}),
    omega(0),
    theta(0),
    torque(0),
    m(m_),
    inv_m(1 / m),
    I(I_),
    inv_I(1 / I),
    e(0.78),
    movable(movable_),
    m_vertices(vertices),
    max_track_length(1e3),
    color{255, 255, 255, 255},
    id(0),
    static_friction(true)
{
    if (!movable) {
        m = INT_MAX;
        I = INT_MAX;
        inv_m = 0;
        inv_I = 0;
        v = Vector2::zero();
    }
}

RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, bool movable_)
:   
#ifdef VERLET
    p_old(pos),
#endif
    a({0, 0}),
    v(vel),
    p(pos),
    f({0, 0}),
    omega(0),
    theta(0),
    torque(0),
    m(m_),
    inv_m(1 / m),
    I(I_),
    inv_I(1 / I),
    e(0.78),
    movable(movable_),
    max_track_length(1e3),
    color{255, 255, 255, 255},
    id(0),
    static_friction(true)
{
    if (!movable) {
        m = INT_MAX;
        I = INT_MAX;
        inv_m = 0;
        inv_I = 0;
        v = Vector2::zero();
    }
}

RigidBody::~RigidBody() {}

void RigidBody::step(double dt) {
    if (!movable)
        return;
#ifdef EX_EULER
    // Explicit Euler scheme
    p += v * dt;
    v += a * dt;
    theta += omega * dt;
    omega += a_theta * dt;
#else
# ifdef IM_EULER
    // Semi implicit Euler scheme
    v += a * dt;
    p += v * dt;
    omega += a_theta * dt;
    theta += omega * dt;
# else 
#   ifdef VERLET
    // Verlet
    Vector2 p_n(p);
    p = p*2 - p_old + a * dt * dt;
    v = (p_n - p_old) / dt;
    p_old = p_n;
#   endif
# endif
#endif /* INTEGRATOR */
}

void RigidBody::apply_newton_second_law() {
    if (movable) {
        a = f / m;
        a_theta = torque / I;
    }else {
        a = Vector2::zero();
        a_theta = 0;
    }
}

void RigidBody::subject_to_force(const Vector2 force) {
    f += force;
}

// NEEDS CORRECTION
void RigidBody::subject_to_torque(const Vector2 force) {
    if (omega <= 3.2) {
        Vector2 r(3 / (double)RENDER_SCALE, 0);
        torque += cross2(r, force);
    }
}

void RigidBody::reset_forces() {
    f = {0, 0};
    torque = 0;
    // body->f -= body->v * 6 * PI * body->r * air_viscosity; // Air resistance
}

void RigidBody::move(const Vector2 delta_p, bool update_AABB) {
    p += delta_p;
    if (update_AABB)
        update_bounding_box();
}

void RigidBody::rotate(const double angle, bool update_AABB) {
    theta += angle;
    if (update_AABB)
        update_bounding_box();
}

void RigidBody::velocity_impulse(const Vector2 impulse) {
    if (movable)
        v += impulse;
}

void RigidBody::angular_impulse(const double impulse) {
    if (movable)
        omega += impulse;
}

double RigidBody::energy(bool gravity) const {
    return k_energy() + p_energy() * gravity;
}

double RigidBody::k_energy() const {
    const double v2(v.x * v.x + v.y * v.y);
    return 0.5 * m * v2 + 0.5 * I * omega * omega;
}

double RigidBody::p_energy() const {
    return movable * m * g * p.y;
}

void RigidBody::draw_forces(SDL_Renderer* renderer) const {
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    render_line(renderer, p.x, p.y, p.x + f.x / 50.0, p.y + f.y / 50.0);
}

void RigidBody::draw_trace(SDL_Renderer* renderer) {
    if (max_track_length > 0) {
        if (track.size() == max_track_length) {
            track.pop_front();
        }
        if (track.size() < max_track_length)
            track.push_back(p);

        for (size_t i(0); i < track.size() - 1; ++i) {
            int alpha(255.0 / track.size() * i);
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, alpha);
            // render_filled_circle(renderer, point.x, point.y, 1 / (double)RENDER_SCALE);
            render_line(renderer, track[i].x, track[i].y, track[i + 1].x, track[i + 1].y);
        }
    }
}

void RigidBody::colorize(const SDL_Color color) {
    this->color = color;
}

void RigidBody::reset_color() {
    color = {255, 255, 255, 255};
}


Ball::Ball(Vector2 vel, Vector2 pos, double m, double r_, bool movable)
:   RigidBody(vel, pos, m, 0.5 * m * r_ * r_, movable),
    r(r_)
{}

std::vector<Vector2> Ball::get_vertices() const {
    // return std::vector<Vector2>{{r, 0}};
    return std::vector<Vector2>{};
}

double Ball::get_radius() const {
    return r;
}

void Ball::draw(SDL_Renderer* renderer) const {
    // render_filled_circle(renderer, p.x, p.y, r);
    // render_fill_circle_fast(renderer, p.x, p.y, r);
    // double mark(0.75 * r);
    // render_filled_circle(renderer, p.x + mark * cos(theta), p.y + mark * sin(theta), r / 5);
    if (movable) {
        SDL_SetRenderDrawColor(renderer, 0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5*color.a);
        render_fill_circle_fast(renderer, p.x, p.y, r);
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        render_circle(renderer, p.x, p.y, r);
        render_line(renderer, p.x, p.y, p.x + r * cos(theta), p.y + r * sin(theta));
    }else {
        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 128);
        render_circle(renderer, p.x, p.y, r);
        render_line(renderer, p.x, p.y, p.x + r * cos(PI / 4.0), p.y + r * sin(PI / 4.0));
        render_line(renderer, p.x, p.y, p.x + r * cos(0.75 * PI), p.y + r * sin(0.75 * PI));
        render_line(renderer, p.x, p.y, p.x + r * cos(0.75 * PI), p.y - r * sin(0.75 * PI));
        render_line(renderer, p.x, p.y, p.x + r * cos(PI / 4.0), p.y - r * sin(PI / 4.0));
    }
    
}

void Ball::handle_wall_collisions() {
    CollisionInfo collision;
    if (p.x - r < 0) {
        collision.normal = {-1, 0};
        collision.depth = r - p.x;
        p.x = r;
        collision.contact_point = {0, p.y};
    }else if (p.x + r > SCENE_WIDTH) {
        collision.normal = {1, 0};
        collision.depth = p.x + r - SCENE_WIDTH;
        p.x = SCENE_WIDTH - r;
        collision.contact_point = {SCENE_WIDTH, p.y};
    }
    if (p.y - r < 0) {
        if (r - p.y > collision.depth) {
            collision.normal = {0, -1};
            collision.contact_point = {p.x, 0};
        }
        p.y = r;
    }else if (p.y + r > SCENE_HEIGHT) {
        if (p.y + r - SCENE_HEIGHT > collision.depth) {
            collision.normal = {0, 1};
            collision.contact_point = {p.x, SCENE_HEIGHT};
        }
        p.y = SCENE_HEIGHT - r;
    }

    if (collision.normal != Vector2::zero()) {
        solve_wall_collision(this, collision);
    }
}

void Ball::update_bounding_box() {
    m_aabb.min = {p.x - r, p.y - r};
    m_aabb.max = {p.x + r, p.y + r};
}

bool Ball::contains_point(const Vector2 point) const {
    return (point - p).norm() <= r;
}


Rectangle::Rectangle(Vector2 vel, Vector2 pos, double m, double w_, double h_, Vertices vertices, 
        bool movable)
:   RigidBody(vel, pos, m, 1/12.0 * m * (w_ * w_ + h_ * h_), movable, vertices),
    w(w_),
    h(h_)
{
    // omega = -PI / 4.0;
    // theta = PI / 4.0;
}

void Rectangle::draw(SDL_Renderer* renderer) const {
        // SDL_SetRenderDrawColor(renderer, 239, 169, 54, 255);
    Vector2 A(m_vertices[0]);
    Vector2 B(m_vertices[1]);
    Vector2 C(m_vertices[2]);
    Vector2 D(m_vertices[3]);

    if (!movable) {
        SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
        render_line(renderer, A.x, A.y, C.x, C.y);
        render_line(renderer, B.x, B.y, D.x, D.y);
    }else {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    }

    render_line(renderer, A.x, A.y, B.x, B.y);
    render_line(renderer, B.x, B.y, C.x, C.y);
    render_line(renderer, C.x, C.y, D.x, D.y);
    render_line(renderer, D.x, D.y, A.x, A.y);
    // SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    // render_line(renderer, m_aabb.min.x, m_aabb.max.y, m_aabb.min.x, m_aabb.min.y);
    // render_line(renderer, m_aabb.min.x, m_aabb.min.y, m_aabb.max.x, m_aabb.min.y);
    // render_line(renderer, m_aabb.max.x, m_aabb.min.y, m_aabb.max.x, m_aabb.max.y);
    // render_line(renderer, m_aabb.max.x, m_aabb.max.y, m_aabb.min.x, m_aabb.max.y);
    // render_line(renderer, 0, 0, m_vertices[0].x, m_vertices[0].y);
    // render_line(renderer, 0, 0, m_vertices[1].x, m_vertices[1].y);
    // render_line(renderer, 0, 0, m_vertices[2].x, m_vertices[2].y);
    // render_line(renderer, 0, 0, m_vertices[3].x, m_vertices[3].y);

    // SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);
    // render_line(renderer, 0, 0, m_t.x / 1.0, m_t.y / 1.0);
}

void Rectangle::handle_wall_collisions() {
    CollisionInfo collision;
    Vector2 r1;
    Vector2 r2;
    int contacts(0);
    if (m_aabb.min.x <= 0) {
        collision.normal = {-1, 0};
        for (auto v : m_vertices) {
            if (v.x <= 0) {
                if (r1 != Vector2::zero()) {
                    r2 = v - p;
                    break;
                }
                r1 = v - p;
            }
        }
        p.x -= m_aabb.min.x;
    }else if (m_aabb.max.x >= SCENE_WIDTH) {
        collision.normal = {1, 0};
        for (auto v : m_vertices) {
            if (v.x >= SCENE_WIDTH) {
                if (r1 != Vector2::zero()) {
                    r2 = v - p;
                    break;
                }
                r1 = v - p;
            }
        }
        p.x -= (m_aabb.max.x - SCENE_WIDTH);
    }
    if (m_aabb.min.y <= 0) {
        collision.normal = {0, -1};
        for (auto v : m_vertices) {
            if (v.y <= 0) {
                if (r1 != Vector2::zero()) {
                    r2 = v - p;
                    break;
                }
                r1 = v - p;
            }
        }
        p.y -= m_aabb.min.y;
    }else if (m_aabb.max.y >= SCENE_HEIGHT) {
        collision.normal = {0, 1};
        for (auto v : m_vertices) {
            if (v.y >= SCENE_HEIGHT) {
                if (r1 != Vector2::zero()) {
                    r2 = v - p;
                    break;
                }
                r1 = v - p;
            }
        }
        p.y -= (m_aabb.max.y - SCENE_HEIGHT);
    }

    if (collision.normal != Vector2::zero()) {
        collision.contact_point = p + r1;
        collision.contact_points.push_back(p + r1);
        if (r2 != Vector2::zero())
            collision.contact_points.push_back(p + r2);
        solve_wall_collision(this, collision);
    }
}

void Rectangle::update_bounding_box() {
    Vector2 A(-w/2 * cos(theta) - h/2 * sin(theta), -w/2 * sin(theta) + h/2 * cos(theta));
    Vector2 B(-w/2 * cos(theta) + h/2 * sin(theta), -w/2 * sin(theta) - h/2 * cos(theta));
    Vector2 C(w/2 * cos(theta) + h/2 * sin(theta), w/2 * sin(theta) - h/2 * cos(theta));
    Vector2 D(w/2 * cos(theta) - h/2 * sin(theta), w/2 * sin(theta) + h/2 * cos(theta));

    m_vertices[0] = p + A;
    m_vertices[1] = p + B;
    m_vertices[2] = p + C;
    m_vertices[3] = p + D;

    std::array<Vector2, 4> local_vertices{A, B, C, D};

    double x_min(0);
    double x_max(0);
    double y_min(0);
    double y_max(0);
    for (auto v : local_vertices) {
        if (v.x > x_max)
            x_max = v.x;
        if (v.x < x_min)
            x_min = v.x;
        if (v.y > y_max)
            y_max = v.y;
        if (v.y < y_min)
            y_min = v.y;
    }

    m_aabb.min = p + Vector2{x_min, y_min};
    m_aabb.max = p + Vector2{x_max, y_max};
}

bool Rectangle::contains_point(const Vector2 point) const {
    // Vector2 u((point - p).rotated(-theta));
    // return abs(u.x) <= w/2.0 && abs(u.y) <= h/2.0;
    Vector2 AP(point - m_vertices[0]);
    Vector2 AB(m_vertices[1] - m_vertices[0]);
    Vector2 AD(m_vertices[3] - m_vertices[0]);
    return (0 <= dot2(AP, AB) && dot2(AP, AB) <= dot2(AB, AB) 
            && 0 <= dot2(AP, AD) && dot2(AP, AD) <= dot2(AD, AD));
}


// Triangle::Triangle(Vector2 vel, Vector2 pos, double m, Vertices vertices, bool movable)
// :   RigidBody(vel, pos, m, )


void Frame::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
    SDL_FRect rect({(float)center.x - (float)width / 2, 
                    (float)SCENE_HEIGHT - (float)center.y - (float)height / 2, 
                    (float)width, 
                    (float)height});
    rect.x *= RENDER_SCALE;
    rect.y *= RENDER_SCALE;
    rect.w *= RENDER_SCALE;
    rect.h *= RENDER_SCALE;

    SDL_RenderDrawRectF(renderer, &rect);
}

