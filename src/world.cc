#include <SDL_hints.h>
#include "world.h"
#include "rigid_body.h"
#include "shape.h"
#include "broad_phase.h"
#include "narrow_phase.h"
#include "collision.h"
#include "utils.h"
#include "render.h"
#include "settings.h"
#include "config.h"

World::World()
:   gravity_enabled(true),
    walls_enabled(true),
    air_friction_enabled(false),
    body_count(0),
    focus(0), 
    m_force_fields({Vector2{0.0, -g * gravity_enabled}, Vector2{0.0, 0.0}, Vector2{0.0, 0.0}})
{
    body_count = m_bodies.size();
    m_profile.reset();
}

World::~World() {
    destroy_all();
}

void World::step(double dt, int substeps, Settings& settings, bool perft) {
    if (perft && body_count < 250) {
        RigidBodyDef def;
        def.position = {0.5 * SCENE_WIDTH, 0.5 * SCENE_HEIGHT};
        def.velocity = {1, 0};
        Shape* ball(create_circle(0.1));
        create_body(def, ball);
    }

    m_profile.reset();
    Timer step_timer;
    Timer broad_timer;
    Timer pairs_timer;
    Timer AABB_timer;
    Timer response_timer;
    Timer walls_timer;

#ifdef SWEEP_AND_PRUNE
    pairs_timer.reset();
    m_sap.choose_axis();
    const auto pairs(m_sap.process());
    m_profile.pairs = pairs_timer.get_microseconds();
    m_profile.broad_phase = m_profile.pairs;
#endif

    for (auto& body : m_bodies) {
        body->reset_color();
    }

    destroy_contacts();
    destroy_proxys();

    for (int i(0); i < substeps; ++i) {
        apply_forces();
        for (auto body : m_bodies) {
            if (body->is_enabled()) {
                Timer ode_timer;
                body->step(dt / substeps);
                // body->update_bounding_box();
                m_profile.ode += ode_timer.get_microseconds();
            }
        }
        
        for (auto& pair : pairs) {
            RigidBody* a(pair[0]);
            RigidBody* b(pair[1]);

            AABB_timer.reset();
            const Shape* shape_a(a->get_shape());
            const Shape* shape_b(b->get_shape());
            bool broad_overlap(AABB_overlap(shape_a->get_aabb(), shape_b->get_aabb()));
            m_profile.AABBs += AABB_timer.get_microseconds();
            m_profile.broad_phase += AABB_timer.get_microseconds();

            if (broad_overlap) {

                Timer narrow_phase_timer;
                Manifold collision(collide(a, b));
                m_profile.narrow_phase += narrow_phase_timer.get_microseconds();

                if (collision.intersecting) {
                    if (i < 2) {
                        m_contacts.push_back(new Manifold(collision));
                    }

                    response_timer.reset();
                    if (!a->is_dynamic()) {
                        b->move(collision.normal * collision.depth);
                    }else if (!b->is_dynamic()) {
                        a->move(-collision.normal * collision.depth);
                    }else {
                        a->move(-collision.normal * collision.depth * 0.5);
                        b->move(collision.normal * collision.depth * 0.5);
                    }
                    solve_collision(a, b, collision);
                    m_profile.response_phase += response_timer.get_microseconds();

                    if (settings.highlight_collisions) {
                        a->colorize({0, 128, 255, 255});
                        b->colorize({0, 255, 128, 255});
                    }
                }else if (i > substeps - 2) {
                    m_proxys.push_back(new DistanceInfo(ditance_convex(shape_a, shape_b)));
                }
            }            
        }

        walls_timer.reset();
        if (walls_enabled) {
            for (auto body : m_bodies) {
                body->handle_wall_collisions();
            }
            m_profile.walls += walls_timer.get_microseconds();
        }
    }

    // m_profile.average((double)steps);
    m_profile.step = step_timer.get_microseconds();
}

void World::render(SDL_Renderer* renderer, bool running, Settings& settings) {
    // Draw world boundaries
    if (walls_enabled) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 127);
        render_line(renderer, {0, SCENE_HEIGHT}, {0, 0});
        render_line(renderer, {0, 0}, {SCENE_WIDTH, 0});
        render_line(renderer, {SCENE_WIDTH, 0}, {SCENE_WIDTH, SCENE_HEIGHT});
        render_line(renderer, {SCENE_WIDTH, SCENE_HEIGHT}, {0, SCENE_HEIGHT});
    }

    if (body_count > 0) {
        m_bodies[focus]->colorize(focus_color);
        if (settings.draw_body_trajectory) {
            m_bodies[focus]->draw_trace(renderer, running);
        }
    }

    for (auto& body : m_bodies) {
        body->draw(renderer);
    }

    if (settings.draw_bounding_boxes) {
        SDL_SetRenderDrawColor(renderer, 102, 102, 255, 255);
        for (auto body : m_bodies) {
            body->draw_bounding_box(renderer);
        }
    }

    if (settings.draw_contact_points) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for (auto contact : m_contacts) {
            for (unsigned i(0); i < contact->count; ++i) {
                const auto point(contact->contact_points[i]);
                render_circle_fill_raster(renderer, point, 3.5 / RENDER_SCALE);
            }
        }
    }

    if (settings.draw_collision_normal) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
        for (auto contact : m_contacts) {
            for (unsigned i(0); i < contact->count; ++i) {
                const auto point(contact->contact_points[i]);
                render_line(renderer, point, point + contact->normal / RENDER_SCALE * 20);
            }
        }
    }

    if (settings.draw_distance_proxys) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        for (auto prox : m_proxys) {
            render_line(renderer, prox->points.closest_a, prox->points.closest_b);
        }
    }

    for (auto spring : m_springs) {
        spring->draw(renderer);
    }
}

RigidBody* World::create_body(const RigidBodyDef& body_def, Shape* shape) {
    RigidBody* body;
    body = new RigidBody(body_def, shape, body_count);

    m_bodies.push_back(body);
    m_sap.update_list(m_bodies);
    ++body_count;

    return body;
}

void World::add_spring(Vector2 p1, Vector2 p2, Spring::DampingType damping, float stiffness) {
    RigidBody* a(nullptr);
    RigidBody* b(nullptr);
    for (auto body : m_bodies) {
        if (body->get_shape()->contains_point(p1)) {
            a = body;
            continue;
        }
        if (body->get_shape()->contains_point(p2)) {
            b = body;
            continue;
        }
        if (a && b) {
            break;
        }
    }
    if (a && b) {
        if ((a->is_dynamic() || b->is_dynamic()) && (a->is_enabled() || b->is_enabled())) {
            const double rest_length((a->get_p() - b->get_p()).norm());
            m_springs.push_back(new Spring(a, b, rest_length, stiffness, damping));
        }
    }
}

void World::destroy_body(RigidBody* body) {
    if (!body || body_count == 0) {
        return;
    }

    int idx(-1);
    for (unsigned i(0); i < body_count; ++i) {
        if (body == m_bodies[i]) {
            idx = i;
            break;
        }
    }

    if (idx >= 0) {
        delete body;
        m_bodies.erase(m_bodies.begin() + idx);
        --body_count;
        if (idx <= focus && focus > 0) {
            --focus;
        }
        m_sap.update_list(m_bodies);
    }
}

void World::destroy_all() {
    for (auto body : m_bodies) {
        delete body;
    }
    m_bodies.clear();
    body_count = 0;
    focus = 0;

    destroy_contacts();
    destroy_proxys();

    for (auto spring : m_springs) {
        delete spring;
    }
    m_springs.clear();

    m_sap.update_list(m_bodies);
}

std::string World::dump_profile() const {
    std::string perf;
    perf += ("Time per step : " + truncate_to_string(m_profile.step) + " us\n")
          + ("ODE solve time : " + truncate_to_string(m_profile.ode) + " us\n")
          + ("Collisions time : " + truncate_to_string(m_profile.collisions) + " us\n")
          + ("  > Broad phase : " + truncate_to_string(m_profile.broad_phase) + " us\n")
          + ("    > Pairs : " + truncate_to_string(m_profile.pairs) + " us\n")
          + ("    > AABBs : " + truncate_to_string(m_profile.AABBs) + " us\n")
          + ("  > Narrow phase : " + truncate_to_string(m_profile.narrow_phase) + " us\n")
#ifdef GJK_EPA
          + ("    > GJK : " + truncate_to_string(m_profile.gjk_collide) + " us\n")
          + ("    > EPA : " + truncate_to_string(m_profile.epa) + " us\n")
#endif
          + ("    > Clip : " + truncate_to_string(m_profile.clip) + " us\n")
          + ("  > Response phase : " + truncate_to_string(m_profile.response_phase) + " us\n")
          + ("Walls : " + truncate_to_string(m_profile.walls) + " us\n");

    return perf;
}

std::string World::dump_selected_body() const {
    std::string body_data;
    if (body_count > 0) {
        body_data = m_bodies[focus]->dump(gravity_enabled);
    }
    return body_data;
}

double World::total_energy() const {
    double energy(0);
    for (auto body : m_bodies) {
        energy += body->energy(gravity_enabled);
    }
    for (auto spring : m_springs) {
        energy += spring->energy();
    }
    return energy;
}

void World::focus_next() {
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

void World::focus_prev() {
    if (body_count == 0) {
        return;
    }

    m_bodies[focus]->reset_color();
    if (focus == 0) {
        focus = body_count - 1;
    }else {
        --focus;
    }
    m_bodies[focus]->colorize(focus_color);
}

void World::focus_on_position(Vector2 p) {
    if (body_count > 0) {
        m_bodies[focus]->reset_color();
    }
    for (size_t i(0); i < body_count; ++i) {
        if (m_bodies[i]->get_shape()->contains_point(p)) {
            focus = i;
            m_bodies[i]->colorize(focus_color);
            break;
        }
    }
}

RigidBody* World::get_focused_body() const {
    if (body_count > 0) {
        return m_bodies[focus];
    }
    return nullptr;
}

void World::apply_forces() {
    for (auto body : m_bodies) {
        body->reset_forces();
        if (gravity_enabled && body->is_enabled() && body->is_dynamic()) {
            body->subject_to_force({0.0, -body->get_mass() * g}, body->get_p());
        }
    }
    for (auto spring : m_springs) {
        spring->apply();
    }
}

Manifold World::collide(RigidBody* body_a, RigidBody* body_b) {
    Manifold result;

    Shape* shape_a(body_a->get_shape());
    Shape* shape_b(body_b->get_shape());
    const ShapeType shape_type_a(shape_a->get_type());
    const ShapeType shape_type_b(shape_b->get_type());

    if (shape_type_a == POLYGON || shape_type_b == POLYGON) {
        Timer gjk, epa, clip;

        result = collide_convex(shape_a, shape_b, gjk, epa, clip);

        m_profile.gjk_collide += gjk.get_microseconds();
        m_profile.epa += epa.get_microseconds();
        m_profile.clip += clip.get_microseconds();
    }else {
        result = collide_circle_circle(shape_a, shape_b);
    }

    return result;
}

void World::destroy_contacts() {
    for (auto contact : m_contacts) {
        delete contact;
    }
    m_contacts.clear();
}

void World::destroy_proxys() {
    for (auto prox : m_proxys) {
        delete prox;
    }
    m_proxys.clear();
}

void World::Profile::reset() {
    this->step = 0;
    this->ode = 0;
    this->collisions = 0;
    this->broad_phase = 0;
    this->pairs = 0;
    this->AABBs = 0;
    this->narrow_phase = 0;
    this->gjk_collide = 0;
    this->epa = 0;
    this->clip = 0;
    this->response_phase = 0;
    this->walls = 0;
}
