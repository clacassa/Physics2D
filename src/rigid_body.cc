#include <iostream>
#include <array>
#include "rigid_body.h"
#include "narrow_phase.h"
#include "collision.h"
#include "utils.h"
#include "render.h"
#include "config.h"
#include "vector2.h"

RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_, 
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
    e(steel_restitution),
    m_type(type_),
    enabled(enabled_),
    m_vertices(vertices),
    max_track_length(1e3),
    id(0),
    static_friction(true)
{
    if (!is_dynamic()) {
        inv_m = 0;
        inv_I = 0;
    }
    if (is_static()) {
        v = Vector2::zero();
    }
    
    reset_color();
}

RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_)
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
    e(steel_restitution),
    m_type(type_),
    enabled(enabled_),
    max_track_length(1e3),
    color{255, 255, 255, 255},
    id(0),
    static_friction(true)
{
    if (!is_dynamic()) {
        inv_m = 0;
        inv_I = 0;
    }  
    if (is_static()) {
        v = Vector2::zero();
    }
    
    reset_color();
}

RigidBody::~RigidBody() {}

void RigidBody::step(double dt) {
    if (is_static()) {
        return;
    }

    // Newton 2nd Law
    a = f / m;
    a_theta = torque / I;

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

void RigidBody::subject_to_force(const Vector2 force) {
    f += force;
}

// NEEDS CORRECTION
void RigidBody::subject_to_torque(const Vector2 world_point, const Vector2 force) {
    const Vector2 r(world_point - p);
    if (contains_point(r)) {
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
    if (update_AABB) {
        update_bounding_box();
    }
}

void RigidBody::rotate(const double d_theta, bool update_AABB) {
    theta += d_theta;
    if (update_AABB) {
        update_bounding_box();
    }
}

void RigidBody::linear_impulse(const Vector2 impulse) {
    if (!is_static()) {
        v += impulse;
    }
}

void RigidBody::angular_impulse(const double impulse) {
    if (!is_static()) {
        omega += impulse;
    }
}

double RigidBody::energy(bool gravity_enabled) const {
    return k_energy() + p_energy() * gravity_enabled;
}

double RigidBody::k_energy() const {
    const double v2(v.x * v.x + v.y * v.y);
    return 0.5 * m * v2 + 0.5 * I * omega * omega;
}

double RigidBody::p_energy() const {
    return is_dynamic() * m * g * p.y;
}

void RigidBody::draw_trace(SDL_Renderer* renderer, bool update_trace) {
    if (max_track_length > 0) {
        if (update_trace) {
            if (track.size() == max_track_length) {
                track.pop_front();
            }
            if (track.size() < max_track_length) {
                track.push_back(p);
            }
        }

        if (track.size() > 1) {
            for (size_t i(0); i < track.size() - 1; ++i) {
                int alpha(255.0 / track.size() * i);
                SDL_SetRenderDrawColor(renderer, 255, 0, 0, alpha);
                // render_circle_fill(renderer, point.x, point.y, 1 / (double)RENDER_SCALE);
                render_line(renderer, track[i], track[i + 1]);
            }
        }
    }
}

void RigidBody::draw_bounding_box(SDL_Renderer* renderer) {
    render_rectangle(renderer, p, m_aabb.max.x - m_aabb.min.x, m_aabb.max.y - m_aabb.min.y);
}

void RigidBody::draw_forces(SDL_Renderer* renderer) const {
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    render_line(renderer, p, p + f / 50.0);
}

void RigidBody::colorize(const SDL_Color color) {
    this->color = color;
}

void RigidBody::reset_color() {
    switch (m_type) {
        case STATIC:
            color = {255, 255, 255, 255};
            break;
        case KINEMATIC:
            color = kinematic_body_color;
            break;
        case DYNAMIC:
            color = dynamic_body_color;
            break;
    }
}

std::string RigidBody::dump(bool gravity_enabled) const {
    std::string body_info("Selected body infos : \n-----------------\n");
    std::string E_m("Mechanical energy : " + truncate_to_string(energy(gravity_enabled)) + " J\n");
    std::string E_k("kinetic energy : " + truncate_to_string(k_energy()) + " J\n");
    std::string E_pp("potential energy : " + truncate_to_string(p_energy()) + " J\n");
    std::string mass("mass : " + truncate_to_string(m) + " kg\n");
    std::string x("x : " + truncate_to_string(p.x) + " m\n");
    std::string y("y : " + truncate_to_string(p.y) + " m\n");
    std::string vx("vx : " + truncate_to_string(v.x) + " m/s\n");
    std::string vy("vy : " + truncate_to_string(v.y) + " m/s\n");
    std::string v_theta("omega : " + truncate_to_string(omega) + " rad/s\n");

    return E_m + mass + x + y + vx + vy + v_theta;
}


Ball::Ball(Vector2 vel, Vector2 pos, double m, double r_, BodyType type, bool enabled)
:   RigidBody(vel, pos, m, 0.5 * m * r_ * r_, type, enabled),
    r(r_)
{
    update_bounding_box();
}

std::vector<Vector2> Ball::get_vertices() const {
    // return std::vector<Vector2>{{r, 0}};
    return std::vector<Vector2>{};
}

double Ball::get_radius() const {
    return r;
}

void Ball::draw(SDL_Renderer* renderer) {
    if (!track.empty()) {
        if (track.back() != p) {
            track.clear();
        }
    }

    if (!is_static() && enabled) {
        SDL_SetRenderDrawColor(renderer, 0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5*color.a);
        render_circle_fill_raster(renderer, p, r);
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        render_circle(renderer, p, r);
        render_line(renderer, p, {p.x + r * cos(theta), p.y + r * sin(theta)});
    }else {
        if (enabled) {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        }else {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a * 0.5);
        }
        render_circle(renderer, p, r);
        if (is_static()) {
            SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a * 0.5);
            render_line(renderer, p, {p.x + r * cos(PI / 4.0), p.y + r * sin(PI / 4.0)});
            render_line(renderer, p, {p.x + r * cos(0.75 * PI), p.y + r * sin(0.75 * PI)});
            render_line(renderer, p, {p.x + r * cos(0.75 * PI), p.y - r * sin(0.75 * PI)});
            render_line(renderer, p, {p.x + r * cos(PI / 4.0), p.y - r * sin(PI / 4.0)});
        }
    }
}

void Ball::handle_wall_collisions() {
    Manifold collision_h, collision_v;  // We separate horizontal and vertical walls collision
    if (p.x - r < 0) {
        collision_h.normal = {-1, 0};
        collision_h.depth = r - p.x;
        p.x = r;
        collision_h.contact_points[0] = {0, p.y};
        collision_h.count = 1;
    }else if (p.x + r > SCENE_WIDTH) {
        collision_h.normal = {1, 0};
        collision_h.depth = p.x + r - SCENE_WIDTH;
        p.x = SCENE_WIDTH - r;
        collision_h.contact_points[0] = {SCENE_WIDTH, p.y};
        collision_h.count = 1;
    }
    if (p.y - r < 0) {
        collision_v.normal = {0, -1};
        collision_v.depth = r - p.y;
        p.y = r;
        collision_v.contact_points[0] = {p.x, 0};
        collision_v.count = 1;
    }else if (p.y + r > SCENE_HEIGHT) {
        collision_v.normal = {0, 1};
        collision_v.depth = p.y + r - SCENE_HEIGHT;
        p.y = SCENE_HEIGHT - r;
        collision_v.contact_points[0] = {p.x, SCENE_HEIGHT};
        collision_v.count = 1;
    }

    if (collision_h.count > 0) {
        solve_wall_collision(this, collision_h);
    }
    if (collision_v.count > 0) {
        solve_wall_collision(this, collision_v);
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
        BodyType type, bool enabled)
:   RigidBody(vel, pos, m, 1/12.0 * m * (w_ * w_ + h_ * h_), type, enabled, vertices),
    w(w_),
    h(h_)
{
    update_bounding_box();
    // omega = -PI / 4.0;
    // theta = PI / 4.0;
}

void Rectangle::draw(SDL_Renderer* renderer) {
    if (!track.empty()) {
        if (track.back() != p) {
            track.clear();
        }
    }
        // SDL_SetRenderDrawColor(renderer, 239, 169, 54, 255);
    Vector2 A(m_vertices[0]);
    Vector2 B(m_vertices[1]);
    Vector2 C(m_vertices[2]);
    Vector2 D(m_vertices[3]);

    if (is_static() && enabled) {
        SDL_SetRenderDrawColor(renderer, 0.75 * color.r, 0.75 * color.g, 0.75 * color.b, color.a);
        render_line(renderer, A, C);
        render_line(renderer, B, D);
    }
    if (enabled) {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    }else {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 0.25 * color.a);
    }
    // Draw outline
    render_line(renderer, A, B);
    render_line(renderer, B, C);
    render_line(renderer, C, D);
    render_line(renderer, D, A);
    // Fill the inside
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, 0.1 * color.a);
    const Vector2 edge(C - B);
    const double len(edge.norm());
    const Vector2 n(edge.normalized());
    const unsigned width_px((len * RENDER_SCALE));
    const double dw(1 / RENDER_SCALE);
    for (unsigned i(0); i < width_px; ++i) {
        render_line(renderer, B + n * i * dw, A + n * i * dw);
    }
}

void Rectangle::handle_wall_collisions() {
    Manifold collision_h, collision_v;
    Vector2 r1_h, r2_h;
    Vector2 r1_v, r2_v;

    if (m_aabb.min.x <= 0) {
        collision_h.normal = {-1, 0};
        for (auto v : m_vertices) {
            if (v.x <= 0) {
                collision_h.contact_points[0 + collision_h.count] = v;
                ++collision_h.count;
                if (collision_h.count >= 2) {
                    break;
                }
            }
        }
        p.x -= m_aabb.min.x;
    }else if (m_aabb.max.x >= SCENE_WIDTH) {
        collision_h.normal = {1, 0};
        for (auto v : m_vertices) {
            if (v.x >= SCENE_WIDTH) {
                collision_h.contact_points[0 + collision_h.count] = v;
                ++collision_h.count;
                if (collision_h.count >= 2) {
                    break;
                }
            }
        }
        p.x -= (m_aabb.max.x - SCENE_WIDTH);
    }
    if (m_aabb.min.y <= 0) {
        collision_v.normal = {0, -1};
        for (auto v : m_vertices) {
            if (v.y <= 0) {
                collision_v.contact_points[0 + collision_v.count] = v;
                ++collision_v.count;
                if (collision_v.count >= 2) {
                    break;
                }
            }
        }
        p.y -= m_aabb.min.y;
    }else if (m_aabb.max.y >= SCENE_HEIGHT) {
        collision_v.normal = {0, 1};
        for (auto v : m_vertices) {
            if (v.y >= SCENE_HEIGHT) {
                collision_v.contact_points[0 + collision_v.count] = v;
                ++collision_v.count;
                if (collision_v.count >= 2) {
                    break;
                }
            }
        }
        p.y -= (m_aabb.max.y - SCENE_HEIGHT);
    }

    //const Vector2 r_h((r1_h + r2_h) * 0.5);
    if (collision_h.count > 0) {
        //collision_h.contact_points.push_back(p + r_h);
        solve_wall_collision(this, collision_h);
    }

    // const Vector2 r_v((r1_v + r2_v) * 0.5);
    if (collision_v.count > 0) {
        // collision_v.contact_points.push_back(p + r_v);
        solve_wall_collision(this, collision_v);
    }
}

void Rectangle::update_bounding_box() {
    Vector2 A(Vector2(-w/2, -h/2).rotated(theta));
    Vector2 B(Vector2(-w/2, h/2).rotated(theta));
    Vector2 C(Vector2(w/2, h/2).rotated(theta));
    Vector2 D(Vector2(w/2, -h/2).rotated(theta));

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
        if (v.x > x_max) {
            x_max = v.x;
        }
        if (v.x < x_min) {
            x_min = v.x;
        }
        if (v.y > y_max) {
            y_max = v.y;
        }
        if (v.y < y_min) {
            y_min = v.y;
        }
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
