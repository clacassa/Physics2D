#include <iostream>
#include "system_state.h"

SystemState::SystemState()
:   gravity(false),
    m_force_fields({Vector2{0.0, -g * gravity}, Vector2{0.0, 0.0}, Vector2{0.0, 0.0}}),
    m_solver(nullptr),
    m_dt(0.0), 
    focus(0) {
    m_bodies.push_back(new RigidBody(5.0, 0.0, SCENE_WIDTH/2, SCENE_HEIGHT/2, 2, 0.1));
}

SystemState::~SystemState() {
    delete m_solver;
    for (auto body : m_bodies) {
        delete body;
    }
    m_bodies.clear();
}

void SystemState::initialize(EulerSolver* solver) {
    m_solver = solver;
}

void SystemState::process(double dt, int steps, bool perft) {
    if (perft)
        add_body(Vector2{SCENE_WIDTH / 2, SCENE_HEIGHT / 2}, Vector2{-5.0, 0.0});

    unsigned long process_time(0);

    for (int i(0); i < steps; ++i) {
        m_solver->start(dt / steps);
        //m_grid.clear_cells();
        //QuadTree qt_root(Rect{500.0, 250.0, 500.0, 250.0});
        //std::vector<RigidBody*> bodies_found;

        while (true) {
            const bool done(m_solver->step(&m_dt));

            auto start(std::chrono::steady_clock::now());

            apply_forces();
            for (auto link : m_fixed_links) {
                link->apply(dt / steps);
            }
            // Newton's Second Law F = ma
            for (auto body : m_bodies) {
                body->a = body->f / body->m;
                body->a_theta = body->torque / body->I;
            }

            for (auto body : m_bodies) {
                m_solver->solve(body, &m_dt);

                // Walls detection
                if (body->p.x - body->r <= 0 || body->p.x + body->r >= SCENE_WIDTH)
                    body->v.x *= -1;
                if (body->p.x - body->r <= 0)
                    body->p.x = body->r;
                if (body->p.x + body->r >= SCENE_WIDTH)
                    body->p.x = SCENE_WIDTH - body->r;

                if (body->p.y - body->r <= 0 || body->p.y + body->r >= SCENE_HEIGHT)
                    body->v.y *= -1;// / (1 + 0.1*body->m);
                if (body->p.y - body->r <= 0)
                    body->p.y = body->r;
                if (body->p.y + body->r >= SCENE_HEIGHT)
                    body->p.y = SCENE_HEIGHT - body->r;

                // Collision detection
                for (unsigned k(0); k < m_bodies.size(); ++k) {
                //  if (m_bodies[k]->p.x == body->p.x && m_bodies[k]->p.y == body->p.y)
                    if (m_bodies[k] == body)
                        continue;
                    if (is_collided(body->p, body->r, m_bodies[k]->p, m_bodies[k]->r)) {
                        Result result(compute_velocity(body, m_bodies[k]));
                        body->v = result.v1;
                        m_bodies[k]->v = result.v2;
                    }
                }
                //qt_root.insert(body);
            }

            // Optimized collision detection
            /*for (auto body : m_bodies) {
                m_grid.update_cell(body, body->p.x, body->p.y);
            }
            m_grid.solve_collisions();*/
            
            /*for (auto& body : m_bodies) {
                const Circle range(body->p.x, body->p.y, body->r * body->r);
                bodies_found.clear();
                qt_root.query(range, bodies_found);
                for (size_t j(0); j < bodies_found.size(); ++j) {
                    if (body != m_bodies[j]) {
                        if (is_collided(body->p, body->r, m_bodies[j]->p, m_bodies[j]->r)) {
                            Result result(compute_velocity(body, m_bodies[j]));
                            body->v = result.v1;
                            m_bodies[j]->v = result.v2;
                        }
                    }
                }
            }
            */

            auto end(std::chrono::steady_clock::now());
            process_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            
            if (done)
                break;
        }
    }
    step_time = process_time / (double)steps;
}

void SystemState::apply_forces() {
    for (auto body : m_bodies) {
        body->f = {0, 0};
        body->torque = 0;
        body->f -= body->v * 6 * PI * body->r * air_viscosity; // Air resistance
    }
    Vector2 f{0, 20};
    Vector2 r(m_bodies[0]->r, 0); // point of application viewed from the center of mass
    if (m_bodies[0]->omega <= 3.2)
        m_bodies[0]->torque += cross2(r, f);

    if (gravity) {
        for (auto body : m_bodies) {
            body->f.y += -body->m * g;
        }
    }

    for (auto c : m_constraints) {
        c->apply();
    }
}

void SystemState::toggle_gravity() {
    gravity = !gravity;
}

void SystemState::add_body(Vector2 pos, Vector2 vel, double mass, double radius) {
    m_bodies.push_back(new RigidBody(vel.x, vel.y, pos.x, pos.y, mass, radius));
    if (gravity)
        m_bodies.back()->f.y = -m_bodies.back()->m * g;
}

void SystemState::add_frame(Vector2 pos) {
    m_frames.push_back(new Frame(pos, 0.5, 0.25));
}

void SystemState::add_link(Vector2 p1, Vector2 p2) {
    RigidBody* a(nullptr);
    RigidBody* b(nullptr);
    Frame* f(nullptr);
    for (auto body : m_bodies) {
        if (Vector2(p1 - body->p).norm() <= body->r) {
            a = body;
            continue;
        }
        if (Vector2(p2 - body->p).norm() <= body->r) {
            b = body;
            continue;
        }
        if (a && b)
            break;
    }
    if (a && b)
        // m_constraints.push_back(new Link(a, b, Vector2(a->p - b->p).norm()));
        m_constraints.push_back(new Spring(a, b, 500, Vector2(a->p - b->p).norm()));
    else if (a) {
        for (auto frame : m_frames) {
            if (abs(Vector2(p2 - frame->center).x) <= frame->width &&
                abs(Vector2(p2 - frame->center).y) <= frame->height) {
                f = frame;
                break;
            }
        }
        m_constraints.push_back(new FixedSpring(a, f, 500, Vector2(a->p - f->center).norm()));
        // m_fixed_links.push_back(new FixedLink(a, f, Vector2(a->p - f->center).norm()));
    }
}

void SystemState::render(SDL_Renderer* renderer) {
    // Scene frame
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer, 1, SCREEN_HEIGHT-1, SCREEN_WIDTH-1, SCREEN_HEIGHT-1);
    SDL_RenderDrawLine(renderer, SCREEN_WIDTH-1, SCREEN_HEIGHT-1, SCREEN_WIDTH-1, 1);

    for (auto body : m_bodies) {
        body->draw(renderer);
        body->draw_forces(renderer);
    }
    // SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    // render_body_circle(renderer, m_bodies[focus]->p.x, m_bodies[focus]->p.y, m_bodies[focus]->r);

    // Render links between objects
    for (auto c : m_constraints) {
        c->draw(renderer);
    }

    // Render frames
    for (auto frame : m_frames) {
        frame->draw(renderer);
    }

    for (auto link : m_fixed_links) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        render_line(renderer, link->a->p.x, link->a->p.y, link->f->center.x, link->f->center.y);
    }
}

std::string SystemState::dump_object_data() const {
    std::string n("Bodies : " + std::to_string(m_bodies.size()) + "\n");
    std::string perf("Time per step : " + std::to_string(step_time) + " us\n");

    double total_energy(0);
    for (auto body : m_bodies) {
        total_energy += body->energy();
    }
    for (auto spring : m_constraints) {
        total_energy += spring->energy();
    }
    std::string system_E_m("System energy : " + std::to_string(total_energy) + " J\n");

    std::string E_m("Mechanical energy : " + std::to_string(m_bodies[focus]->energy()) + " J\n");
    std::string E_k("kinetic energy : " + std::to_string(m_bodies[focus]->k_energy()) + " J\n");
    std::string E_pp("potential energy : " + std::to_string(m_bodies[focus]->p_energy()) + " J\n");
    std::string m("mass : " + std::to_string(m_bodies[focus]->m) + " kg\n");
    std::string x("x : " + std::to_string(m_bodies[focus]->p.x) + " m\n");
    std::string y("y : " + std::to_string(m_bodies[focus]->p.y) + " m\n");
    std::string vx("vx : " + std::to_string(m_bodies[focus]->v.x) + " m/s\n");
    std::string vy("vy : " + std::to_string(m_bodies[focus]->v.y) + " m/s\n");

    return "\n" + n + perf + system_E_m + "\n" + E_m + E_k + E_pp;
}

void SystemState::focus_next() {
    ++focus;
    if (focus >= m_bodies.size())
        focus = 0;
}

void SystemState::focus_prev() {
    if (focus == 0)
        focus = m_bodies.size() - 1;
    else
        --focus;
}


