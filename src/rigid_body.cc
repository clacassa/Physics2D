#include <SDL_stdinc.h>
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

RigidBody::RigidBody(const RigidBodyDef& def, const Shape& shape, const size_t id)
:
    m_pos(def.position),
    m_vel(def.velocity),
    m_theta(def.rotation),
    m_omega(def.angular_velocity),
    m_density(def.density),
    m_restitution(def.restitution),
    m_type(def.type),
    m_enabled(def.enabled),
    // m_shape(shape),
    m_id(id)
{
    switch (shape.get_type()) {
        case CIRCLE:
            m_shape = new Circle(shape.get_radius());
            break;
        case POLYGON:
            ConvexHull hull;
            hull.points = shape.get_vertices();
            hull.count = shape.get_count();
            m_shape = new Polygon(hull);
            break;
    }

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

RigidBody::RigidBody(const Shape& shape, const size_t id)
:
    m_theta(0),
    m_omega(0),
    m_density(steel_density),
    m_restitution(steel_restitution),
    m_type(DYNAMIC),
    m_enabled(true),
    m_id(id)
{
    switch (shape.get_type()) {
        case CIRCLE:
            m_shape = new Circle(shape.get_radius());
            break;
        case POLYGON:
            ConvexHull hull;
            hull.points = shape.get_vertices();
            hull.count = shape.get_count();
            m_shape = new Polygon(hull);
            break;
    }

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

    SDL_Color color;
    color.r = m_color.r * 0.5;
    color.g = m_color.g * 0.5;
    color.b = m_color.b * 0.5;
    color.a = m_color.a * 0.5;

    if (m_shape->get_type() == CIRCLE) {
        const double r(m_shape->get_radius());

        if (m_type != STATIC && m_enabled) {
            m_shape->draw(renderer, color, true);
            color = m_color;
            m_shape->draw(renderer, color, false);

            const Vector2 indicator(m_pos.x + r * cos(m_theta), m_pos.y + r * sin(m_theta));
            render_line(renderer, m_pos, indicator);
        }else {
            if (m_enabled) {
                color = m_color;
                // SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a);
            }else {
                color = m_color;
                color.a *= 0.5;
                // SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
            }
            m_shape->draw(renderer, color, false);
            if (m_type == STATIC) {
                SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, m_color.a * 0.5);
                render_line(renderer, m_pos, m_pos + vector2_q1 * r);
                render_line(renderer, m_pos, m_pos + vector2_q2 * r);
                render_line(renderer, m_pos, m_pos + vector2_q3 * r);
                render_line(renderer, m_pos, m_pos + vector2_q4 * r);
            }
        }
    }else {
        if (m_type == STATIC && m_enabled) {
            SDL_SetRenderDrawColor(renderer, 0.75 * m_color.r, 0.75 * m_color.g, 0.75 * m_color.b, m_color.a);

            const Vertices vertices(m_shape->get_vertices());
            if (m_shape->get_count() == 4) {
                render_line(renderer, vertices[0], vertices[2]);
                render_line(renderer, vertices[1], vertices[3]);
            }
        }
        color = m_color;
        if (!m_enabled) {
            color.a *= 0.25;
        }
        // Draw outline
        m_shape->draw(renderer, color, false);

        // Fill the inside
        // color.a = m_color.a * 0.1;
        // SDL_SetRenderDrawColor(renderer, m_color.r, m_color.g, m_color.b, 0.1 * m_color.a);
        // const Vector2 edge(C - B);
        // const double len(edge.norm());
        // const Vector2 n(edge.normalized());
        // const unsigned width_px((len * RENDER_SCALE));
        // const double dw(1 / RENDER_SCALE);
        // for (unsigned i(0); i < width_px; ++i) {
        //     render_line(renderer, B + n * i * dw, A + n * i * dw);
        // }
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
    render_line(renderer, aabb.min, {aabb.max.x, aabb.min.y});
    render_line(renderer, {aabb.max.x, aabb.min.y}, aabb.max);
    render_line(renderer, aabb.max, {aabb.min.x, aabb.max.y});
    render_line(renderer, {aabb.min.x, aabb.max.y}, aabb.min);
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
        const uint8_t count(m_shape->get_count());

        if (aabb.min.x <= 0) {
            collision_h.normal = {-1, 0};
            for (uint8_t i(0); i < count; ++i) {
                if (vertices[i].x <= 0) {
                    collision_h.contact_points[0 + collision_h.count] = vertices[i];
                    ++collision_h.count;
                    if (collision_h.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.x -= aabb.min.x;
        }else if (aabb.max.x >= SCENE_WIDTH) {
            collision_h.normal = {1, 0};
            for (uint8_t i(0); i < count; ++i) {
                if (vertices[i].x >= SCENE_WIDTH) {
                    collision_h.contact_points[0 + collision_h.count] = vertices[i];
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
            for (uint8_t i(0); i < count; ++i) {
                if (vertices[i].y <= 0) {
                    collision_v.contact_points[0 + collision_v.count] = vertices[i];
                    ++collision_v.count;
                    if (collision_v.count >= 2) {
                        break;
                    }
                }
            }
            m_pos.y -= aabb.min.y;
        }else if (aabb.max.y >= SCENE_HEIGHT) {
            collision_v.normal = {0, 1};
            for (uint8_t i(0); i < count; ++i) {
                if (vertices[i].y >= SCENE_HEIGHT) {
                    collision_v.contact_points[0 + collision_v.count] = vertices[i];
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
