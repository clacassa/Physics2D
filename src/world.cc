#include <SDL_hints.h>
#include <cstddef>
#include <iostream>
#include <cassert>
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
#include "vector2.h"

World::World()
:   gravity_enabled(1),
    walls_enabled(0),
    air_friction_enabled(0),
    body_count(0),
    focus(-1)
{
    m_bodies.reserve(500);
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
        Circle ball(0.1);
        add_body(def, ball);
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

    destroy_contacts();
    destroy_proxys();

    for (int i(0); i < substeps; ++i) {
        apply_forces();
        for (auto spring : m_springs) {
            spring->apply(dt);
        }
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

            if (a->get_type() == STATIC && b->get_type() == STATIC) {
                continue;
            }

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

    if (body_count > 0 && focus >= 0) {
        m_bodies[focus]->colorize(focus_color);
        if (settings.draw_body_trajectory) {
            m_bodies[focus]->draw_trace(renderer, running);
        }
    }

    for (auto& body : m_bodies) {
        body->draw(renderer);
        body->reset_color();
    }

    if (settings.draw_center_of_mass) {
        for (auto body :m_bodies) {
            body->draw_com(renderer);
        }
    }

    if (settings.draw_bounding_boxes) {
        SDL_SetRenderDrawColor(renderer, 178, 102, 255, 255);
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

RigidBody* World::add_body(const RigidBodyDef& body_def, const Shape& shape) {
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

void World::add_force_field(const Vector2 field) {
    m_force_fields.push_back(field);
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
    focus = -1;

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
    perf += ("Time per step : " + truncate_to_string(m_profile.step / 1e3) + " ms\n")
          + ("ODE solve time : " + truncate_to_string(m_profile.ode / 1e3) + " ms\n")
          + ("Collisions time : " + truncate_to_string(m_profile.collisions / 1e3) + " ms\n")
          + ("  > Broad phase : " + truncate_to_string(m_profile.broad_phase / 1e3) + " ms\n")
          + ("    > Pairs : " + truncate_to_string(m_profile.pairs / 1e3) + " ms\n")
          + ("    > AABBs : " + truncate_to_string(m_profile.AABBs / 1e3) + " ms\n")
          + ("  > Narrow phase : " + truncate_to_string(m_profile.narrow_phase / 1e3) + " ms\n")
#ifdef GJK_EPA
          + ("    > GJK : " + truncate_to_string(m_profile.gjk_collide / 1e3) + " ms\n")
          + ("    > EPA : " + truncate_to_string(m_profile.epa / 1e3) + " ms\n")
#endif
          + ("    > Clip : " + truncate_to_string(m_profile.clip / 1e3) + " ms\n")
          + ("  > Response phase : " + truncate_to_string(m_profile.response_phase / 1e3) + " ms\n")
          + ("Walls : " + truncate_to_string(m_profile.walls / 1e3) + " ms\n");

    return perf;
}

std::string World::dump_selected_body() const {
    std::string body_data;
    if (body_count > 0 && focus >= 0) {
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

bool World::focus_next() {
    if (body_count == 0) {
        return false;
    }

    const int previous_focus(focus);
    if (body_count > 0 && focus >= 0) {
        m_bodies[focus]->reset_color();
    }
    ++focus;
    if (focus >= body_count) {
        focus = 0;
    }
    m_bodies[focus]->colorize(focus_color);

    return previous_focus != focus;
}

bool World::focus_prev() {
    if (body_count == 0) {
        return false;
    }

    const int previous_focus(focus);
    if (body_count > 0 && focus >= 0) {
        m_bodies[focus]->reset_color();
    }
    if (focus <= 0) {
        focus = body_count - 1;
    }else {
        --focus;
    }
    m_bodies[focus]->colorize(focus_color);

    return previous_focus != focus;
}

bool World::focus_on_position(Vector2 p) {
    if (body_count > 0 && focus >= 0) {
        m_bodies[focus]->reset_color();
    }
    const int previous_focus(focus);
    for (size_t i(0); i < body_count; ++i) {
        if (m_bodies[i]->get_shape()->contains_point(p)) {
            focus = i;
            m_bodies[i]->colorize(focus_color);
            break;
        }
    }

    return previous_focus != focus;
}

bool World::focus_at(const int index) {
    if (index == -1) {
        focus = -1;
        return true;
    }

   assert(index >= 0 && index < body_count);
   if (body_count > 0 && focus >= 0) {
       m_bodies[focus]->reset_color();
   }
   const int previous_focus(focus);
   focus = index;
   return previous_focus != focus;
}

bool World::focus_body(const RigidBody* body) {
    bool focus_changed(0);
    for (unsigned i(0); i < body_count; ++i) {
        if (body == m_bodies[i]) {
            if (focus != (int)i) {
                focus_changed = 1;
            }
            focus = i;
            break;
        }
    }
    return focus_changed;
}

RigidBody* World::get_focused_body() const {
    if (body_count > 0 && focus >= 0) {
        return m_bodies[focus];
    }
    return nullptr;
}

RigidBody* World::get_body_at(const size_t index) const {
    assert(index >= 0 && index < body_count);
    return m_bodies[index];
}

Spring* World::get_spring_from_mouse(Vector2 p) {
    for (auto spring : m_springs) {
        const Vector2 axis(spring->get_axis());
        const Vector2 n(axis.normal());
        const Vector2 p1(spring->get_anchor());
        const Vector2 p2(p1 - axis);
        const double w(0.125); // 20 pixels wide hitbox
        Vertices points = {
            p1 - n * w,
            p2 - n * w,
            p2 + n * w,
            p1 + n * w
        };
        Polygon hitbox({points, 4});
        if (hitbox.contains_point(p)) {
            return spring;
        }
    }
    return nullptr;
}

Spring* World::get_spring_at(const size_t index) const {
    assert(index >= 0);
    if (index < m_springs.size()) {
        return m_springs[index];
    }
    return nullptr;
}

void World::apply_forces() {
    for (auto body : m_bodies) {
        const Vector2 cm(body->get_p());
        const double m(body->get_mass());
        body->reset_forces();
        if (gravity_enabled) {
            body->subject_to_force({0.0, -m * g}, cm);
        }
        if (air_friction_enabled) {
            body->subject_to_force(-body->get_v() * 10 * air_viscosity, cm);
        }
        for (auto f : m_force_fields) {
            body->subject_to_force(f * m, cm);
        }
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
