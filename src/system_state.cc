#include <iostream>
#include "SDL_render.h"
#include "system_state.h"
#include "rigid_body.h"
#include "collision.h"
#include "utils.h"
#include "render.h"
#include "config.h"

using namespace std::chrono;

SystemState::SystemState()
:   gravity_enabled(true),
    air_friction_enabled(false),
    body_count(0),
    focus(0), 
    m_force_fields({Vector2{0.0, -g * gravity_enabled}, Vector2{0.0, 0.0}, Vector2{0.0, 0.0}})
{
    //this->add_ball({SCENE_WIDTH / 2.0, SCENE_HEIGHT / 2.0}, 2.0, SCENE_WIDTH / 100, DYNAMIC, true, {5.0, 0.0});
    body_count = m_bodies.size();
    m_Time_Perf.reset();
}

SystemState::~SystemState() {
    for (auto body : m_bodies) {
        delete body;
    }
    m_bodies.clear();

    for (auto contact : m_contacts) {
        delete contact;
    }
    m_contacts.clear();

    for (auto spring : m_springs) {
        delete spring;
    }
    m_springs.clear();
}

void SystemState::process(double dt, int steps, bool perft) {
    if (perft && body_count < 250) {
        add_ball({SCENE_WIDTH/2.0, SCENE_HEIGHT/2.0}, 1.0);
    }

    m_Time_Perf.reset();
    
    for (auto contact : m_contacts) {
        delete contact;
    }
    m_contacts.clear();

#ifdef SWEEP_AND_PRUNE
    m_SAP.choose_axis(m_bodies);
#endif

    for (auto& body : m_bodies) {
        body->reset_color();
    }

    for (int i(0); i < steps; ++i) {
        auto start(steady_clock::now());

        apply_forces();
        for (auto body : m_bodies) {
            if (body->is_enabled()) {
                auto t0(steady_clock::now());
                body->step(dt / steps);
                body->update_bounding_box();
                auto t1(steady_clock::now());
                m_Time_Perf.ode_time += duration_cast<microseconds>(t1 - t0).count();
            }
        }

        auto t2(steady_clock::now());
        
#ifdef SWEEP_AND_PRUNE
        auto result(m_SAP.process(m_bodies));
        auto t3(steady_clock::now());
        m_Time_Perf.broad_phase += duration_cast<microseconds>(t3 - t2).count();

        for (auto& pair : result) {
            RigidBody* a(pair[0]);
            RigidBody* b(pair[1]);

            auto t4(steady_clock::now());
            bool broad_overlap(AABB_overlap(a->get_AABB(), b->get_AABB()));
            auto t5(steady_clock::now());
            m_Time_Perf.broad_phase += duration_cast<microseconds>(t5 - t4).count();

            if (broad_overlap) {

                auto t6(steady_clock::now());
                Manifold collision(detect_collision(a, b, m_Time_Perf.GJK, m_Time_Perf.EPA));
                auto t7(steady_clock::now());

                if (collision.intersecting) {
                    m_contacts.push_back(new Manifold(collision));

                    auto t8(steady_clock::now());
                    if (a->is_static()) {
                        b->move(collision.normal * collision.depth);
                    }else if (b->is_static()) {
                        a->move(-collision.normal * collision.depth);
                    }else {
                        a->move(-collision.normal * collision.depth * 0.5);
                        b->move(collision.normal * collision.depth * 0.5);
                    }
                    solve_collision(a, b, collision);
                    auto t9(steady_clock::now());
                    m_Time_Perf.response_phase += duration_cast<microseconds>(t9 - t8).count();
#ifdef DEBUG_COLLISION
                    a->colorize({0, 128, 255, 255});
                    b->colorize({0, 255, 128, 255});
#endif /* DEBUG_COLLISION */
                }
                m_Time_Perf.narrow_phase += duration_cast<microseconds>(t7 - t6).count();
            }            
        }
#else
        for (unsigned j(0); j < body_count - 1; ++j) {
            for (unsigned k(j + 1); k < body_count; ++k) {
                RigidBody* a(m_bodies[j]);
                RigidBody* b(m_bodies[k]);

                auto t10(steady_clock::now());
                bool broad_overlap(AABB_overlap(a->get_AABB(), b->get_AABB()));
                auto t11(steady_clock::now());
                m_Time_Perf.broad_phase += duration_cast<microseconds>(t11 - t10).count();

                if (broad_overlap) {

                    auto t12(steady_clock::now());
                    Manifold collision(detect_collision(a,b, m_Time_Perf.GJK,m_Time_Perf.EPA));
                    auto t13(steady_clock::now());

                    if (collision.intersecting) {

                        auto t14(steady_clock::now());
                        if (a->is_static()) {
                            b->move(collision.normal * collision.depth);
                        }else if (b->is_static()) {
                            a->move(-collision.normal * collision.depth);
                        }else {
                            a->move(-collision.normal * collision.depth * 0.5);
                            b->move(collision.normal * collision.depth * 0.5);
                        }
                        solve_collision(m_bodies[j], m_bodies[k], collision);
                        auto t15(steady_clock::now());
                        m_Time_Perf.response_phase += duration_cast<microseconds>(t15-t14).count();
#ifdef DEBUG_COLLISION
                        a->colorize({0, 128, 255, 255});
                        b->colorize({0, 255, 128, 255});
#endif /* DEBUG_COLLISION */
                    }
                    m_Time_Perf.narrow_phase += duration_cast<microseconds>(t13 - t12).count();
                }
            }
        }
#endif /* SWEEP_AND_PRUNE */
        auto end(steady_clock::now());

        for (auto body : m_bodies) {
            if (!body->is_static()) {
                //body->handle_wall_collisions();
            }
        }

        m_Time_Perf.step_time += duration_cast<microseconds>(end - start).count();
        m_Time_Perf.collisions_time += duration_cast<microseconds>(end - t2).count();
    }
    m_Time_Perf.average((double)steps);
    // m_Time_Perf.ode_time /= body_count;
}

void SystemState::apply_forces() {
    for (auto body : m_bodies) {
        body->reset_forces();
        if (gravity_enabled && body->is_enabled()) {
            body->subject_to_force({0.0, -body->get_mass() * g});
        }
    }
    for (auto spring : m_springs) {
        spring->apply();
    }
}

void SystemState::toggle_gravity() {
    gravity_enabled = !gravity_enabled;
}

void SystemState::destroy_all() {
    for (auto body : m_bodies) {
        delete body;
    }
    m_bodies.clear();

    for (auto contact : m_contacts) {
        delete contact;
    }
    m_contacts.clear();

    for (auto spring : m_springs) {
        delete spring;
    }
    m_springs.clear();

    m_SAP.process(m_bodies);
}

void SystemState::add_ball(Vector2 pos, double radius, BodyType type, bool enabled,
        Vector2 vel) {
    const double depth(radius);
    const double volume(PI * radius * radius * depth);
    m_bodies.push_back(new Ball(vel, pos, stl_steel_density * volume, radius, type, enabled));
    ++body_count;
}

void SystemState::add_rectangle(Vector2 p, double w, double h, BodyType type, bool enabled, Vector2 v) {
    const std::vector<Vector2> vertices{
        {p.x - w / 2, p.y + h / 2},
        {p.x - w / 2, p.y - h / 2},
        {p.x + w / 2, p.y - h / 2},
        {p.x + w / 2, p.y + h / 2}
    };
    m_bodies.push_back(new Rectangle(v, p, stl_steel_density * w * h * h, w, h, vertices, type, enabled));
    ++body_count;
}

void SystemState::add_spring(Vector2 p1, Vector2 p2, Spring::DampingType damping, float stiffness) {
    RigidBody* a(nullptr);
    RigidBody* b(nullptr);
    for (auto body : m_bodies) {
        if (body->contains_point(p1)) {
            a = body;
            continue;
        }
        if (body->contains_point(p2)) {
            b = body;
            continue;
        }
        if (a && b) {
            break;
        }
    }
    if (a && b) {
        if ((!a->is_static() || !b->is_static()) && (a->is_enabled() || b->is_enabled())) {
            const double rest_length((a->get_p() - b->get_p()).norm());
            m_springs.push_back(new Spring(a, b, rest_length, stiffness, damping));
        }
    }
}

void SystemState::move_focused_body(Vector2 delta_p) {
    if (body_count > 0) {
        m_bodies[focus]->move(delta_p);
    }
}

void SystemState::rotate_focused_body(double angle) {
    if (body_count > 0) {
        m_bodies[focus]->rotate(angle);
    }
}

void SystemState::render(SDL_Renderer* renderer, bool running, bool draw_body_motion) {
    // Draw world boundaries
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 127);
    render_line(renderer, {0, SCENE_HEIGHT}, {0, 0});
    render_line(renderer, {0, 0}, {SCENE_WIDTH, 0});
    render_line(renderer, {SCENE_WIDTH, 0}, {SCENE_WIDTH, SCENE_HEIGHT});
    render_line(renderer, {SCENE_WIDTH, SCENE_HEIGHT}, {0, SCENE_HEIGHT});

    if (body_count > 0) {
        m_bodies[focus]->colorize(focus_color);
        if (draw_body_motion) {
            m_bodies[focus]->draw_trace(renderer, running);
        }
    }

    for (auto& body : m_bodies) {
        body->draw(renderer);
#ifdef DEBUG
        //body->draw_forces(renderer);
#endif
    }
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    for (auto contact : m_contacts) {
        render_fill_circle_fast(renderer, contact->contact_points[0], 2.5 / RENDER_SCALE);
        render_fill_circle_fast(renderer, contact->contact_points[1], 2.5 / RENDER_SCALE);
    }
    for (auto spring : m_springs) {
        spring->draw(renderer);
    }
}

std::string SystemState::dump_metrics() const {
    std::string perf;
    perf += ("Time per step : " + truncate_to_string(m_Time_Perf.step_time) + " us\n")
          + ("ODE solve time : " + truncate_to_string(m_Time_Perf.ode_time) + " us\n")
          + ("Collisions time : " + truncate_to_string(m_Time_Perf.collisions_time) + " us\n")
          + ("  > Broad phase : " + truncate_to_string(m_Time_Perf.broad_phase) + " us\n")
          + ("  > Narrow phase : " + truncate_to_string(m_Time_Perf.narrow_phase) + " us\n");
#ifdef GJK_EPA
    perf += ("    > GJK : " + truncate_to_string(m_Time_Perf.GJK) + " us\n")
          + ("    > EPA : " + truncate_to_string(m_Time_Perf.EPA) + " us\n");
#endif
    perf += ("  > Response phase : " + truncate_to_string(m_Time_Perf.response_phase) + " us\n");

    return perf;
}

std::string SystemState::dump_selected_body() const {
    std::string body_data;
    if (body_count > 0) {
        body_data = m_bodies[focus]->dump(gravity_enabled);
#ifdef DEBUG
        body_data += "\nFriction: " + m_bodies[focus]->get_friction();
#endif
    }
    return body_data;
}

double SystemState::total_energy() const {
    double energy(0);
    for (auto body : m_bodies) {
        energy += body->energy(gravity_enabled);
    }
    for (auto spring : m_springs) {
        energy += spring->energy();
    }
    return energy;
}

void SystemState::focus_next() {
    if (body_count == 0) {
        return;
    }

    m_bodies[focus]->reset_color();
    ++focus;
    if (focus >= body_count) {
        focus = 0;
    }
    m_bodies[focus]->colorize(focus_color);
}

void SystemState::focus_prev() {
    if (body_count == 0) {
        return;
    }

    m_bodies[focus]->reset_color();
    if (focus == 0) {
        focus = body_count - 1;
    }else
        --focus;
    m_bodies[focus]->colorize(focus_color);
}

void SystemState::focus_on_position(Vector2 p) {
    if (body_count > 0) {
        m_bodies[focus]->reset_color();
    }
    for (size_t i(0); i < body_count; ++i) {
        if (m_bodies[i]->contains_point(p)) {
            focus = i;
            m_bodies[i]->colorize(focus_color);
            break;
        }
    }
}

RigidBody* SystemState::get_selected_body() const {
    if (body_count > 0) {
        return m_bodies[focus];
    }
    return nullptr;
}

void SystemState::TimePerf::reset() {
    this->step_time = 0;
    this->ode_time = 0;
    this->collisions_time = 0;
    this->broad_phase = 0;
    this->narrow_phase = 0;
    this->GJK = 0;
    this->EPA = 0;
    this->response_phase = 0;
}

void SystemState::TimePerf::average(double steps) {
    this->step_time /= steps;
    this->ode_time /= steps;
    this->collisions_time /= steps;
    this->broad_phase /= steps;
    this->narrow_phase /= steps;
    this->GJK /= steps;
    this->EPA /= steps;
    this->response_phase /= steps;
}


