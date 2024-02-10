#include <iostream>
#include "system_state.h"
#include "collision.h"
#include "utils.h"
#include "config.h"

using namespace std::chrono;

SystemState::SystemState()
:   gravity(true),
    air_friction_enabled(false),
    m_force_fields({Vector2{0.0, -g * gravity}, Vector2{0.0, 0.0}, Vector2{0.0, 0.0}}),
    focus(0) {
    this->add_ball({SCENE_WIDTH / 2.0, SCENE_HEIGHT / 2.0}, 2.0, 0.1, true, {5.0, 0.0});
    // this->add_rectangle({SCENE_WIDTH / 4.0, SCENE_HEIGHT / 2.0}, 1, 0.3, 0.15);
    // this->add_rectangle({0.75 * SCENE_WIDTH, SCENE_HEIGHT / 2.0}, 1, 0.3, 0.15);
    m_bodies[focus]->colorize(SDL_Color{128, 0, 255, 255});
}

SystemState::~SystemState() {
    for (auto body : m_bodies) {
        delete body;
    }
    m_bodies.clear();
}

void SystemState::process(double dt, int steps, bool perft) {
    if (perft)
        add_ball({SCENE_WIDTH/2.0, SCENE_HEIGHT/2.0}, 1.0, 0.1);

    m_Time_Perf.reset();

#ifdef SWEEP_AND_PRUNE
    m_SAP.choose_axis(m_bodies);
#endif

    for (auto& body : m_bodies) {
        body->reset_color();
    }

    for (int i(0); i < steps; ++i) {
        // m_solver->start(dt / steps);
        auto start(steady_clock::now());

        apply_forces();
        for (auto body : m_bodies) {
            body->apply_newton_second_law();
            // m_solver->solve(body, &m_dt);
            auto t0(steady_clock::now());
            body->step(dt / steps);
            auto t1(steady_clock::now());
            m_Time_Perf.ode_time += duration_cast<microseconds>(t1 - t0).count();
            body->update_bounding_box();
        }

        auto t2(steady_clock::now());
        
#ifdef SWEEP_AND_PRUNE
        std::vector<std::vector<RigidBody*>> result(m_SAP.process(m_bodies));
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
                CollisionInfo collision(detect_collision(a, b, m_Time_Perf.GJK, m_Time_Perf.EPA));
                auto t7(steady_clock::now());

                if (collision.intersecting) {

                    auto t8(steady_clock::now());
                    if (!a->is_movable())
                        b->move(collision.normal * collision.depth);
                    else if (!b->is_movable())
                        a->move(-collision.normal * collision.depth);
                    else {
                        a->move(collision.normal * -collision.depth * 0.5);
                        b->move(collision.normal * collision.depth * 0.5);
                    }
                    solve_collision(a, b, collision);
                    auto t9(steady_clock::now());
                    m_Time_Perf.response_phase += duration_cast<microseconds>(t9 - t8).count();

                    // a->colorize({0, 0, 255, 255});
                    // b->colorize({0, 255, 0, 255});
                }
                m_Time_Perf.narrow_phase += duration_cast<microseconds>(t7 - t6).count();
            }            
        }
#else
        for (unsigned j(0); j < m_bodies.size() - 1; ++j) {
            for (unsigned k(j + 1); k < m_bodies.size(); ++k) {
                RigidBody* a(m_bodies[j]);
                RigidBody* b(m_bodies[k]);

                auto t10(steady_clock::now());
                bool broad_overlap(AABB_overlap(a->get_AABB(), b->get_AABB()));
                auto t11(steady_clock::now());
                m_Time_Perf.broad_phase += duration_cast<microseconds>(t11 - t10).count();

                if (broad_overlap) {

                    auto t12(steady_clock::now());
                    CollisionInfo collision(detect_collision(a,b, m_Time_Perf.GJK,m_Time_Perf.EPA));
                    auto t13(steady_clock::now());

                    if (collision.intersecting) {
                        auto t14(steady_clock::now());
                        a->move(collision.normal * -collision.depth * 0.5);
                        b->move(collision.normal * collision.depth * 0.5);
                        // solve_collision(m_bodies[j], m_bodies[k], collision);
                        auto t15(steady_clock::now());
                    }
                    m_Time_Perf.narrow_phase += duration_cast<microseconds>(t13 - t12).count();
                }
            }
        }
#endif /* SWEEP_AND_PRUNE */
        auto end(steady_clock::now());

        for (auto body : m_bodies) {
            body->handle_wall_collisions();
        }

        m_Time_Perf.step_time += duration_cast<microseconds>(end - start).count();
        m_Time_Perf.collisions_time += duration_cast<microseconds>(end - t2).count();
    }
    m_Time_Perf.average((double)steps);
    m_Time_Perf.ode_time /= m_bodies.size();
}

void SystemState::apply_forces() {
    for (auto body : m_bodies) {
        body->reset_forces();
        if (gravity)
            body->subject_to_force({0.0, -body->get_mass() * g});
    }

    for (auto c : m_constraints) {
        c->apply();
    }
}

void SystemState::toggle_gravity() {
    gravity = !gravity;
}

void SystemState::add_ball(Vector2 pos, double mass, double radius, bool movable, Vector2 vel) {
    m_bodies.push_back(new Ball(vel, pos, mass, radius, movable));
}

void SystemState::add_rectangle(Vector2 p, double m, double w, double h, bool movable, Vector2 v) {
    const std::vector<Vector2> vertices{
        {p.x - w / 2, p.y + h / 2},
        {p.x - w / 2, p.y - h / 2},
        {p.x + w / 2, p.y - h / 2},
        {p.x + w / 2, p.y + h / 2}
    };
    m_bodies.push_back(new Rectangle(v, p, m, w, h, vertices, movable));
}

void SystemState::add_frame(Vector2 pos) {
    m_frames.push_back(new Frame(pos, 0.5, 0.25));
}

void SystemState::add_link(Vector2 p1, Vector2 p2) {
    RigidBody* a(nullptr);
    RigidBody* b(nullptr);
    Frame* f(nullptr);
    for (auto body : m_bodies) {
        if (body->contains_point(p1)) {
            a = body;
            continue;
        }
        if (body->contains_point(p2)) {
            b = body;
            continue;
        }
        if (a && b)
            break;
    }
    if (a && b)
        // m_constraints.push_back(new Link(a, b, Vector2(a->get_p() - b->get_p()).norm()));
        m_constraints.push_back(new Spring(a, b, 1e6, (a->get_p() - b->get_p()).norm()));
    else if (a) {
        for (auto frame : m_frames) {
            if (abs(Vector2(p2 - frame->center).x) <= frame->width &&
                abs(Vector2(p2 - frame->center).y) <= frame->height) {
                f = frame;
                break;
            }
        }
        if (f)
            m_constraints.push_back(new FixedSpring(a, f, 1e6   , (a->get_p() - f->center).norm()));
            // m_constraints.push_back(new FixedLink(a, f, Vector2(a->get_p() - f->center).norm()));
    }
}

void SystemState::move_focused_body(Vector2 delta_p) {
    m_bodies[focus]->move(delta_p);
}

void SystemState::rotate_focused_body(double angle) {
    m_bodies[focus]->rotate(angle);
}

void SystemState::render(SDL_Renderer* renderer) {
    // Scene frame
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
    SDL_RenderDrawLine(renderer, 1, SCREEN_HEIGHT-1, SCREEN_WIDTH-1, SCREEN_HEIGHT-1);
    SDL_RenderDrawLine(renderer, SCREEN_WIDTH-1, SCREEN_HEIGHT-1, SCREEN_WIDTH-1, 1);

    m_bodies[focus]->colorize({128, 0, 255, 255});
    for (auto& body : m_bodies) {
        body->draw(renderer);
        // body->draw_forces(renderer);
        // body->draw_trace(renderer);
    }
    
    // Render links between objects
    for (auto c : m_constraints) {
        c->draw(renderer);
    }

    // Render frames
    for (auto frame : m_frames) {
        frame->draw(renderer);
    }
}

std::string SystemState::dump_data() const {
    std::string n("Bodies : " + std::to_string(m_bodies.size()) + "\n");
    std::string perf;
    perf += ("Time per step : " + std::to_string(m_Time_Perf.step_time) + " us\n")
          + ("ODE solve time : " + std::to_string(m_Time_Perf.ode_time) + " us\n")
          + ("Collisions solve time : " + std::to_string(m_Time_Perf.collisions_time) + " us\n")
          + ("  > Broad phase : " + std::to_string(m_Time_Perf.broad_phase) + " us\n")
          + ("  > Narrow phase : " + std::to_string(m_Time_Perf.narrow_phase) + " us\n");
#ifdef GJK_EPA
    perf += ("    > GJK : " + std::to_string(m_Time_Perf.GJK) + " us\n")
          + ("    > EPA : " + std::to_string(m_Time_Perf.EPA) + " us\n");
#endif
    perf += ("  > Response phase : " + std::to_string(m_Time_Perf.response_phase) + " us\n");

    double total_energy(0);
    for (auto body : m_bodies) {
        total_energy += body->energy(gravity);
    }
    for (auto spring : m_constraints) {
        total_energy += spring->energy();
    }
    std::string system_E_m("System energy : " + std::to_string(total_energy) + " J\n");

    const RigidBody* body(m_bodies[focus]);
    std::string body_info("Body informations : \n-----------------\n");
    std::string E_m("Mechanical energy : " + std::to_string(body->energy(gravity)) + " J\n");
    std::string E_k("kinetic energy : " + std::to_string(body->k_energy()) + " J\n");
    std::string E_pp("potential energy : " + std::to_string(body->p_energy()) + " J\n");
    std::string m("mass : " + truncate_to_string(body->get_mass(), 1000) + " kg\n");
    std::string x("x : " + truncate_to_string(body->get_p().x, 1000) + " m\n");
    std::string y("y : " + truncate_to_string(body->get_p().y, 1000) + " m\n");
    std::string vx("vx : " + truncate_to_string(body->get_v().x, 1000) + " m/s\n");
    std::string vy("vy : " + truncate_to_string(body->get_v().y, 1000) + " m/s\n");
    std::string omega("omega : " + truncate_to_string(body->get_omega(), 1000));
    omega += " rad/s\n";
    body_info += E_m + m + x + y + vx + vy + omega;
#ifdef DEBUG_FRICTION
    body_info += "\n" + m_bodies[focus]->get_friction();
#endif

    return "\n" + n + perf + "\n" + system_E_m + "\n" + body_info;
}

void SystemState::focus_next() {
    m_bodies[focus]->reset_color();
    ++focus;
    if (focus >= m_bodies.size())
        focus = 0;
    m_bodies[focus]->colorize(SDL_Color{128, 0, 255, 255});
}

void SystemState::focus_prev() {
    m_bodies[focus]->reset_color();
    if (focus == 0)
        focus = m_bodies.size() - 1;
    else
        --focus;
    m_bodies[focus]->colorize(SDL_Color{128, 0, 255, 255});
}

void SystemState::focus_on_position(Vector2 p) {
    m_bodies[focus]->reset_color();
    for (size_t i(0); i < m_bodies.size(); ++i) {
        if (m_bodies[i]->contains_point(p)) {
            focus = i;
            m_bodies[i]->colorize(SDL_Color{128, 0, 255, 255});
            break;
        }
    }
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


