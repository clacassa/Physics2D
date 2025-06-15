#include <iostream>
#include <array>
#include "rigid_body.h"
#include "shape.h"
#include "narrow_phase.h"
#include "collision.h"
#include "shape.h"
#include "utils.h"
#include "render.h"
#include "config.h"
#include "vector2.h"

RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_, 
        Vertices vertices)
:   
    m_acc({0, 0}),
    m_vel(vel),
    m_pos(pos),
    m_force({0, 0}),
    m_omega(0),
    m_theta(0),
    m_torque(0),
    m_mass(m_),
    m_inv_mass(1 / m_mass),
    m_inertia(I_),
    m_inv_inertia(1 / m_inertia),
    m_restitution(steel_restitution),
    m_type(type_),
    m_enabled(enabled_),
    max_track_length(1e3),
    m_id(0)
{
    if (m_type != DYNAMIC) {
        m_inv_mass = 0;
        m_inv_inertia = 0;
    }
    if (m_type == STATIC) {
        m_vel = vector2_zero;
    }
    
    reset_color();
}

RigidBody::RigidBody(Vector2 vel, Vector2 pos, double m_, double I_, BodyType type_, bool enabled_)
:   
    m_acc({0, 0}),
    m_vel(vel),
    m_pos(pos),
    m_force({0, 0}),
    m_omega(0),
    m_theta(0),
    m_torque(0),
    m_mass(m_),
    m_inv_mass(1 / m_mass),
    m_inertia(I_),
    m_inv_inertia(1 / m_inertia),
    m_restitution(steel_restitution),
    m_type(type_),
    m_enabled(enabled_),
    max_track_length(1e3),
    m_color{255, 255, 255, 255},
    m_id(0)
{
    if (m_type != DYNAMIC) {
        m_inv_mass = 0;
        m_inv_inertia = 0;
    }  
    if (m_type == STATIC) {
        m_vel = vector2_zero;
    }
    
    reset_color();
}

RigidBody::RigidBody(const RigidBodyDef& def, Shape* shape)
:
    m_pos(def.position),
    m_vel(def.velocity),
    m_theta(def.rotation),
    m_omega(def.angular_velocity),
    m_density(def.density),
    m_restitution(def.restitution),
    m_type(def.type),
    m_enabled(def.enabled),
    m_shape(shape)
{
    const MassProperties mp(m_shape->compute_mass_properties(m_density));
    m_mass = mp.mass;
    m_inertia = mp.inertia;

    if (m_type != DYNAMIC) {
        m_inv_mass = 0;
        m_inv_inertia = 0;
    }else {
        m_inv_mass = 1.0 / m_mass;
        m_inv_inertia = 1.0 / m_inertia;
    }
    if (m_type == STATIC) {
        m_vel = vector2_zero;
        m_omega = 0;
    }

    m_shape->transform(m_pos, m_theta);
    reset_color();
}


RigidBody::~RigidBody() {}

void RigidBody::step(double dt) {
    if (m_type == STATIC) {
        return;
    }

    // Newton 2nd Law
    m_acc = m_force / m_mass;
    m_alpha = m_torque / m_inertia;

#ifdef EX_EULER
    // Explicit Euler scheme
    m_pos += m_vel * dt;
    m_vel += m_acc * dt;
    m_theta += m_omega * dt;
    m_omega += m_alpha * dt;
#elif defined IM_EULER
    // Semi implicit Euler scheme
    m_vel += m_acc * dt;
    m_pos += m_vel * dt;
    m_omega += m_alpha * dt;
    m_theta += m_omega * dt;
#endif /* INTEGRATOR */

    m_shape->transform(m_pos, m_theta);
}

void RigidBody::subject_to_force(const Vector2 force, const Vector2 point) {
    if (m_type != DYNAMIC) {
        return;
    }

    m_force += force;
    // if (contains_point(point)) {
    //     const Vector2 r(point - m_pos);
    //     m_torque += cross2(r, force);
    // }
}

void RigidBody::subject_to_torque(const double torque) {
    if (m_type != DYNAMIC) {
        return;
    }

    m_torque += torque;
}

void RigidBody::reset_forces() {
    m_force = vector2_zero;
    m_torque = 0;
    // body->m_force -= body->m_vel * 6 * PI * body->r * air_viscosity; // Air resistance
}

void RigidBody::move(const Vector2 delta_p, bool update_AABB) {
    m_pos += delta_p;
    // if (update_AABB) {
    //     update_bounding_box();
    // }
    m_shape->transform(m_pos, m_theta);
}

void RigidBody::rotate(const double d_theta, bool update_AABB) {
    m_theta += d_theta;
    // if (update_AABB) {
    //     update_bounding_box();
    // }
    // m_shape->rotate(d_theta);
    m_shape->transform(m_pos, m_theta);
}

void RigidBody::linear_impulse(const Vector2 impulse) {
    if (m_type != DYNAMIC) {
        return;
    }

    m_vel += impulse;
}

void RigidBody::angular_impulse(const double impulse) {
    if (m_type != DYNAMIC) {
        return;
    }

    m_omega += impulse;
}

void RigidBody::set_linear_vel(const Vector2 vel) {
    if (m_type == STATIC) {
        return;
    }

    m_vel = vel;
}

void RigidBody::set_angular_vel(const double omega) {
    if (m_type == STATIC) {
        return;
    }

    m_omega = omega;
}

double RigidBody::energy(bool gravity_enabled) const {
    return k_energy() + p_energy() * gravity_enabled;
}

double RigidBody::k_energy() const {
    const double v2(m_vel.x * m_vel.x + m_vel.y * m_vel.y);
    return 0.5 * m_mass * v2 + 0.5 * m_inertia * m_omega * m_omega;
}

double RigidBody::p_energy() const {
    if (m_type != DYNAMIC) {
        return 0;
    }

    return m_mass * g * m_pos.y;
}

void RigidBody::draw(SDL_Renderer* renderer) {
    if (!track.empty()) {
        if (track.back() != m_pos) {
            track.clear();
        }
    }

    if (m_shape->get_type() == CIRCLE) {
        const double r(m_shape->get_radius());

        if (m_type != STATIC && m_enabled) {
            SDL_SetRenderDrawColor(renderer, 0.5 * m_color.r, 0.5 * m_color.g, 0.5 * m_color.b, 0.5*m_color.a);
            render_circle_fill_raster(renderer, m_pos, r);
            SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
            render_circle(renderer, m_pos, r);
            render_line(renderer, m_pos, {m_pos.x + r * cos(m_theta), m_pos.y + r * sin(m_theta)});
        }else {
            if (m_enabled) {
                SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
            }else {
                SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
            }
            render_circle(renderer, m_pos, r);
            if (m_type == STATIC) {
                SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
                render_line(renderer, m_pos, {m_pos.x + r * cos(PI / 4.0), m_pos.y + r * sin(PI / 4.0)});
                render_line(renderer, m_pos, {m_pos.x + r * cos(0.75 * PI), m_pos.y + r * sin(0.75 * PI)});
                render_line(renderer, m_pos, {m_pos.x + r * cos(0.75 * PI), m_pos.y - r * sin(0.75 * PI)});
                render_line(renderer, m_pos, {m_pos.x + r * cos(PI / 4.0), m_pos.y - r * sin(PI / 4.0)});
            }
        }
    }else {
        const Vertices vertices(m_shape->get_vertices());
        Vector2 A(vertices[0]);
        Vector2 B(vertices[1]);
        Vector2 C(vertices[2]);
        Vector2 D(vertices[3]);

        if (m_type == STATIC && m_enabled) {
            SDL_SetRenderDrawColor(renderer, 0.75 * m_color.r, 0.75 * m_color.g, 0.75 * m_color.b, m_color.a);
            render_line(renderer, A, C);
            render_line(renderer, B, D);
        }
        if (m_enabled) {
            SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
        }else {
            SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, 0.25 * m_color.a);
        }
        // Draw outline
        render_line(renderer, A, B);
        render_line(renderer, B, C);
        render_line(renderer, C, D);
        render_line(renderer, D, A);
        // Fill the inside
        SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, 0.1 * m_color.a);
        const Vector2 edge(C - B);
        const double len(edge.norm());
        const Vector2 n(edge.normalized());
        const unsigned width_px((len * RENDER_SCALE));
        const double dw(1 / RENDER_SCALE);
        for (unsigned i(0); i < width_px; ++i) {
            render_line(renderer, B + n * i * dw, A + n * i * dw);
        }
    }

#ifdef DEBUG
    // Debug drawings
    //
#endif
}

void RigidBody::draw_trace(SDL_Renderer* renderer, bool update_trace) {
    if (max_track_length > 0) {
        if (update_trace) {
            if (track.size() == max_track_length) {
                track.pop_front();
            }
            if (track.size() < max_track_length) {
                track.push_back(m_pos);
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
    AABB aabb(m_shape->get_aabb());
    render_rectangle(renderer, m_pos, aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y);
}

void RigidBody::draw_forces(SDL_Renderer* renderer) const {
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    render_line(renderer, m_pos, m_pos + m_force / 50.0);
}

void RigidBody::colorize(const SDL_Color color) {
    this->m_color = color;
    
}

void RigidBody::reset_color() {
    switch (m_type) {
        case STATIC:
            m_color = {255, 255, 255, 255};
            break;
        case KINEMATIC:
            m_color = kinematic_body_color;
            break;
        case DYNAMIC:
            m_color = dynamic_body_color;
            break;
    }
}

std::string RigidBody::dump(bool gravity_enabled) const {
    std::string body_info("Selected body infos : \n-----------------\n");
    std::string E_m("Mechanical energy : " + truncate_to_string(energy(gravity_enabled)) + " J\n");
    std::string E_k("kinetic energy : " + truncate_to_string(k_energy()) + " J\n");
    std::string E_pp("potential energy : " + truncate_to_string(p_energy()) + " J\n");
    std::string mass("mass : " + truncate_to_string(m_mass) + " kg\n");
    std::string x("x : " + truncate_to_string(m_pos.x) + " m\n");
    std::string y("y : " + truncate_to_string(m_pos.y) + " m\n");
    std::string vx("vx : " + truncate_to_string(m_vel.x) + " m/s\n");
    std::string vy("vy : " + truncate_to_string(m_vel.y) + " m/s\n");
    std::string v_theta("m_omega : " + truncate_to_string(m_omega) + " rad/s\n");

    return E_m + mass + x + y + vx + vy + v_theta;
}

void RigidBody::handle_wall_collisions() {
    if (m_type != DYNAMIC) {
        return;
    }

    Manifold collision_h, collision_v;  // We separate horizontal and vertical walls

    if (m_shape->get_type() == CIRCLE) {

        const double r(m_shape->get_radius());

        if (m_pos.x - r < 0) {
            collision_h.normal = {-1, 0};
            collision_h.depth = r - m_pos.x;
            m_pos.x = r;
            collision_h.contact_points[0] = {0, m_pos.y};
            collision_h.count = 1;
        }else if (m_pos.x + r > SCENE_WIDTH) {
            collision_h.normal = {1, 0};
            collision_h.depth = m_pos.x + r - SCENE_WIDTH;
            m_pos.x = SCENE_WIDTH - r;
            collision_h.contact_points[0] = {SCENE_WIDTH, m_pos.y};
            collision_h.count = 1;
        }
        if (m_pos.y - r < 0) {
            collision_v.normal = {0, -1};
            collision_v.depth = r - m_pos.y;
            m_pos.y = r;
            collision_v.contact_points[0] = {m_pos.x, 0};
            collision_v.count = 1;
        }else if (m_pos.y + r > SCENE_HEIGHT) {
            collision_v.normal = {0, 1};
            collision_v.depth = m_pos.y + r - SCENE_HEIGHT;
            m_pos.y = SCENE_HEIGHT - r;
            collision_v.contact_points[0] = {m_pos.x, SCENE_HEIGHT};
            collision_v.count = 1;
        }

        if (collision_h.count > 0) {
            solve_wall_collision(this, collision_h);
        }
        if (collision_v.count > 0) {
            solve_wall_collision(this, collision_v);
        }
    }else {
        Vector2 r1_h, r2_h;
        Vector2 r1_v, r2_v;

        const AABB aabb(m_shape->get_aabb());
        const Vertices vertices(m_shape->get_vertices());

        if (aabb.min.x <= 0) {
            collision_h.normal = {-1, 0};
            for (auto v : vertices) {
                if (v.x <= 0) {
                    collision_h.contact_points[0 + collision_h.count] = v;
                    ++collision_h.count;
                    if (collision_h.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.x -= aabb.min.x;
        }else if (aabb.max.x >= SCENE_WIDTH) {
            collision_h.normal = {1, 0};
            for (auto v : vertices) {
                if (v.x >= SCENE_WIDTH) {
                    collision_h.contact_points[0 + collision_h.count] = v;
                    ++collision_h.count;
                    if (collision_h.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.x -= (aabb.max.x - SCENE_WIDTH);
        }
        if (aabb.min.y <= 0) {
            collision_v.normal = {0, -1};
            for (auto v : vertices) {
                if (v.y <= 0) {
                    collision_v.contact_points[0 + collision_v.count] = v;
                    ++collision_v.count;
                    if (collision_v.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.y -= aabb.min.y;
        }else if (aabb.max.y >= SCENE_HEIGHT) {
            collision_v.normal = {0, 1};
            for (auto v : vertices) {
                if (v.y >= SCENE_HEIGHT) {
                    collision_v.contact_points[0 + collision_v.count] = v;
                    ++collision_v.count;
                    if (collision_v.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.y -= (aabb.max.y - SCENE_HEIGHT);
        }

        //const Vector2 r_h((r1_h + r2_h) * 0.5);
        if (collision_h.count > 0) {
            //collision_h.contact_points.push_back(m_pos + r_h);
            solve_wall_collision(this, collision_h);
        }

        // const Vector2 r_v((r1_v + r2_v) * 0.5);
        if (collision_v.count > 0) {
            // collision_v.contact_points.push_back(m_pos + r_v);
            solve_wall_collision(this, collision_v);
        }
    }

    m_shape->transform(m_pos, m_theta);
}


// Ball::Ball(Vector2 vel, Vector2 pos, double m, double r_, BodyType type, bool enabled)
// :   RigidBody(vel, pos, m, 0.5 * m * r_ * r_, type, enabled)
// {
//     m_shape->transform(m_pos, m_theta);
// }

// std::vector<Vector2> Ball::get_vertices() const {
//     // return std::vector<Vector2>{{r, 0}};
//     return std::vector<Vector2>{};
// }

// double Ball::get_radius() const {
//     return r;
// }

// void Ball::draw(SDL_Renderer* renderer) {
//     if (!track.empty()) {
//         if (track.back() != m_pos) {
//             track.clear();
//         }
//     }
//
//     const double r(m_shape->get_radius());
//
//     if (m_type != STATIC && m_enabled) {
//         SDL_SetRenderDrawColor(renderer, 0.5 * m_color.r, 0.5 * m_color.g, 0.5 * m_color.b, 0.5*m_color.a);
//         render_circle_fill_raster(renderer, m_pos, r);
//         SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
//         render_circle(renderer, m_pos, r);
//         render_line(renderer, m_pos, {m_pos.x + r * cos(m_theta), m_pos.y + r * sin(m_theta)});
//     }else {
//         if (m_enabled) {
//             SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
//         }else {
//             SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
//         }
//         render_circle(renderer, m_pos, r);
//         if (m_type == STATIC) {
//             SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
//             render_line(renderer, m_pos, {m_pos.x + r * cos(PI / 4.0), m_pos.y + r * sin(PI / 4.0)});
//             render_line(renderer, m_pos, {m_pos.x + r * cos(0.75 * PI), m_pos.y + r * sin(0.75 * PI)});
//             render_line(renderer, m_pos, {m_pos.x + r * cos(0.75 * PI), m_pos.y - r * sin(0.75 * PI)});
//             render_line(renderer, m_pos, {m_pos.x + r * cos(PI / 4.0), m_pos.y - r * sin(PI / 4.0)});
//         }
//     }
// }

// void Ball::handle_wall_collisions() {
//     if (m_type != DYNAMIC) {
//         return;
//     }
//
//     const double r(m_shape->get_radius());
//
//     Manifold collision_h, collision_v;  // We separate horizontal and vertical walls collision
//     if (m_pos.x - r < 0) {
//         collision_h.normal = {-1, 0};
//         collision_h.depth = r - m_pos.x;
//         m_pos.x = r;
//         collision_h.contact_points[0] = {0, m_pos.y};
//         collision_h.count = 1;
//     }else if (m_pos.x + r > SCENE_WIDTH) {
//         collision_h.normal = {1, 0};
//         collision_h.depth = m_pos.x + r - SCENE_WIDTH;
//         m_pos.x = SCENE_WIDTH - r;
//         collision_h.contact_points[0] = {SCENE_WIDTH, m_pos.y};
//         collision_h.count = 1;
//     }
//     if (m_pos.y - r < 0) {
//         collision_v.normal = {0, -1};
//         collision_v.depth = r - m_pos.y;
//         m_pos.y = r;
//         collision_v.contact_points[0] = {m_pos.x, 0};
//         collision_v.count = 1;
//     }else if (m_pos.y + r > SCENE_HEIGHT) {
//         collision_v.normal = {0, 1};
//         collision_v.depth = m_pos.y + r - SCENE_HEIGHT;
//         m_pos.y = SCENE_HEIGHT - r;
//         collision_v.contact_points[0] = {m_pos.x, SCENE_HEIGHT};
//         collision_v.count = 1;
//     }
//
//     if (collision_h.count > 0) {
//         solve_wall_collision(this, collision_h);
//     }
//     if (collision_v.count > 0) {
//         solve_wall_collision(this, collision_v);
//     }
// }

// void Ball::update_bounding_box() {
//     m_aabb.min = {m_pos.x - r, m_pos.y - r};
//     m_aabb.max = {m_pos.x + r, m_pos.y + r};
// }

// bool Ball::contains_point(const Vector2 point) const {
//     return (point - m_pos).norm() <= r;
// }


// Rectangle::Rectangle(Vector2 vel, Vector2 pos, double m, double w_, double h_, Vertices vertices,
//         BodyType type, bool enabled)
// :   RigidBody(vel, pos, m, 1/12.0 * m * (w_ * w_ + h_ * h_), type, enabled, vertices)
// {
//     // update_bounding_box();
//     // m_omega = -PI / 4.0;
//     // m_theta = PI / 4.0;
// }

// void Rectangle::draw(SDL_Renderer* renderer) {
//     if (!track.empty()) {
//         if (track.back() != m_pos) {
//             track.clear();
//         }
//     }
//
//         // SDL_SetRenderDrawColor(renderer, 239, 169, 54, 255);
//     const Vertices vertices(m_shape->get_vertices());
//     Vector2 A(vertices[0]);
//     Vector2 B(vertices[1]);
//     Vector2 C(vertices[2]);
//     Vector2 D(vertices[3]);
//
//     if (m_type == STATIC && m_enabled) {
//         SDL_SetRenderDrawColor(renderer, 0.75 * m_color.r, 0.75 * m_color.g, 0.75 * m_color.b, m_color.a);
//         render_line(renderer, A, C);
//         render_line(renderer, B, D);
//     }
//     if (m_enabled) {
//         SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
//     }else {
//         SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, 0.25 * m_color.a);
//     }
//     // Draw outline
//     render_line(renderer, A, B);
//     render_line(renderer, B, C);
//     render_line(renderer, C, D);
//     render_line(renderer, D, A);
//     // Fill the inside
//     SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, 0.1 * m_color.a);
//     const Vector2 edge(C - B);
//     const double len(edge.norm());
//     const Vector2 n(edge.normalized());
//     const unsigned width_px((len * RENDER_SCALE));
//     const double dw(1 / RENDER_SCALE);
//     for (unsigned i(0); i < width_px; ++i) {
//         render_line(renderer, B + n * i * dw, A + n * i * dw);
//     }
// }

// void Rectangle::handle_wall_collisions() {
//     if (m_type != DYNAMIC) {
//         return;
//     }
//
//     Manifold collision_h, collision_v;
//     Vector2 r1_h, r2_h;
//     Vector2 r1_v, r2_v;
//
//     const AABB aabb(m_shape->get_aabb());
//     const Vertices vertices(m_shape->get_vertices());
//
//     if (aabb.min.x <= 0) {
//         collision_h.normal = {-1, 0};
//         for (auto v : vertices) {
//             if (v.x <= 0) {
//                 collision_h.contact_points[0 + collision_h.count] = v;
//                 ++collision_h.count;
//                 if (collision_h.count >= 2) {
//                     break;
//                 }
//             }
//         }
//         m_pos.x -= aabb.min.x;
//     }else if (aabb.max.x >= SCENE_WIDTH) {
//         collision_h.normal = {1, 0};
//         for (auto v : vertices) {
//             if (v.x >= SCENE_WIDTH) {
//                 collision_h.contact_points[0 + collision_h.count] = v;
//                 ++collision_h.count;
//                 if (collision_h.count >= 2) {
//                     break;
//                 }
//             }
//         }
//         m_pos.x -= (aabb.max.x - SCENE_WIDTH);
//     }
//     if (aabb.min.y <= 0) {
//         collision_v.normal = {0, -1};
//         for (auto v : vertices) {
//             if (v.y <= 0) {
//                 collision_v.contact_points[0 + collision_v.count] = v;
//                 ++collision_v.count;
//                 if (collision_v.count >= 2) {
//                     break;
//                 }
//             }
//         }
//         m_pos.y -= aabb.min.y;
//     }else if (aabb.max.y >= SCENE_HEIGHT) {
//         collision_v.normal = {0, 1};
//         for (auto v : vertices) {
//             if (v.y >= SCENE_HEIGHT) {
//                 collision_v.contact_points[0 + collision_v.count] = v;
//                 ++collision_v.count;
//                 if (collision_v.count >= 2) {
//                     break;
//                 }
//             }
//         }
//         m_pos.y -= (aabb.max.y - SCENE_HEIGHT);
//     }
//
//     //const Vector2 r_h((r1_h + r2_h) * 0.5);
//     if (collision_h.count > 0) {
//         //collision_h.contact_points.push_back(m_pos + r_h);
//         solve_wall_collision(this, collision_h);
//     }
//
//     // const Vector2 r_v((r1_v + r2_v) * 0.5);
//     if (collision_v.count > 0) {
//         // collision_v.contact_points.push_back(m_pos + r_v);
//         solve_wall_collision(this, collision_v);
//     }
//
//     m_shape->transform(m_pos, m_theta);
// }

// void Rectangle::update_bounding_box() {
//     Vector2 A(Vector2(-w/2, -h/2).rotated(m_theta));
//     Vector2 B(Vector2(-w/2, h/2).rotated(m_theta));
//     Vector2 C(Vector2(w/2, h/2).rotated(m_theta));
//     Vector2 D(Vector2(w/2, -h/2).rotated(m_theta));
//
//     m_vertices[0] = m_pos + A;
//     m_vertices[1] = m_pos + B;
//     m_vertices[2] = m_pos + C;
//     m_vertices[3] = m_pos + D;
//
//     std::array<Vector2, 4> local_vertices{A, B, C, D};
//
//     double x_min(0);
//     double x_max(0);
//     double y_min(0);
//     double y_max(0);
//     for (auto v : local_vertices) {
//         if (v.x > x_max) {
//             x_max = v.x;
//         }
//         if (v.x < x_min) {
//             x_min = v.x;
//         }
//         if (v.y > y_max) {
//             y_max = v.y;
//         }
//         if (v.y < y_min) {
//             y_min = v.y;
//         }
//     }
//
//     m_aabb.min = m_pos + Vector2{x_min, y_min};
//     m_aabb.max = m_pos + Vector2{x_max, y_max};
// }

// bool Rectangle::contains_point(const Vector2 point) const {
//     // Vector2 u((point - m_pos).rotated(-m_theta));
//     // return abs(u.x) <= w/2.0 && abs(u.y) <= h/2.0;
//     Vector2 AP(point - m_vertices[0]);
//     Vector2 AB(m_vertices[1] - m_vertices[0]);
//     Vector2 AD(m_vertices[3] - m_vertices[0]);
//     return (0 <= dot2(AP, AB) && dot2(AP, AB) <= dot2(AB, AB)
//             && 0 <= dot2(AP, AD) && dot2(AP, AD) <= dot2(AD, AD));
// }


// Triangle::Triangle(Vector2 vel, Vector2 pos, double m, Vertices vertices, bool movable)
// :   RigidBody(vel, pos, m, )
