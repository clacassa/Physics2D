#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <array>
#include <climits>
#include <cassert>
#include "collision.h"
#include "narrow_phase.h"
#include "rigid_body.h"
#include "config.h"

using namespace std::chrono;


void solve_collision(RigidBody* a, RigidBody* b, const Manifold& collision) {
    assert(collision.count <= 2);

    const Vector2 n(collision.normal);
    std::array<double, 2> impulse_list;
    std::array<Vector2, 2> friction_list;
    std::array<Vector2, 2> ra_list;
    std::array<Vector2, 2> rb_list;

    for (uint8_t i(0); i < collision.count; ++i) {
        const Vector2 p(collision.contact_points[i]);

        Vector2 ra(p - a->get_p());
        Vector3 ra_3(ra.x, ra.y, 0);
        Vector3 wa_3(0, 0, a->get_omega());
        Vector2 v_pa(a->get_v() + Vector2(cross(wa_3, ra_3).x, cross(wa_3, ra_3).y));

        Vector2 rb(p - b->get_p());
        Vector3 rb_3(rb.x, rb.y, 0);
        Vector3 wb_3(0, 0, b->get_omega());
        Vector2 v_pb(b->get_v() + Vector2(cross(wb_3, rb_3).x, cross(wb_3, rb_3).y));

        Vector2 v_r(v_pb - v_pa);

        Vector2 u(triple_product(-ra, ra, n) * a->get_inv_I()
                + triple_product(-rb, rb, n) * b->get_inv_I());

        double denom(a->get_inv_m() + b->get_inv_m() + dot2(u, n));
        double impulse(-(1 + std::min(a->get_cor(), b->get_cor())) * dot2(v_r, n) / denom);

        impulse /= collision.count;
        // Vector2 j(n * impulse);

#ifdef FRICTION
        const double vr_n(dot2(v_r, n));
        Vector2 f_e(b->get_f());
        const double fe_n(dot2(f_e, n));
        Vector2 t;
        if (vr_n != 0) {
            t = (v_r - n * vr_n).normalized();
        }else if (fe_n != 0) {
            t = (f_e - n * fe_n).normalized();
        }

        double j_s(0.7 * impulse); // 0.78 for stainless steel
        double j_d(0.42 * impulse); // 0.42 for stainless steel
        // double friction(-dot2(v_r, t) / (1 / a->get_mass() + 1 / b->get_mass() + dot2(u, t)));
        double friction(dot2(v_r, t) / (a->get_inv_m() + b->get_inv_m()));

        friction /= collision.count;

        Vector2 j_t(t * friction);
        Vector2 j_f;
        if (abs(friction) <= j_s) {
            j_f = -j_t;
#ifdef DEBUG
            a->set_friction_debug(true);
            b->set_friction_debug(true);
#endif
        }else {
            j_f = -t * j_d;
#ifdef DEBUG
            a->set_friction_debug(false);
            b->set_friction_debug(false);
#endif
        }
        friction_list[i] = j_f;
#endif /* FRICTION */
        impulse_list[i] = impulse;
        ra_list[i] = ra;
        rb_list[i] = rb;
    }

    for (uint8_t i(0); i < collision.count; ++i) {
        double impulse(impulse_list[i]);
        Vector2 j(n * impulse);
        a->linear_impulse(-j * a->get_inv_m());
        b->linear_impulse(j * b->get_inv_m());

        Vector2 ra(ra_list[i]); 
        a->angular_impulse(-impulse * a->get_inv_I() * cross2(ra, n));
        Vector2 rb(rb_list[i]);
        b->angular_impulse(impulse * b->get_inv_I() * cross2(rb, n));

#ifdef FRICTION
        Vector2 j_f(friction_list[i]);
        const float attenuation_a(b->is_dynamic() ? 1 : 0.5);
        const float attenuation_b(a->is_dynamic() ? 1 : 0.5);

        a->linear_impulse(-j_f * a->get_inv_m() * 1);
        b->linear_impulse(j_f * b->get_inv_m() * 1);

        a->angular_impulse(-cross2(ra, j_f) * a->get_inv_I() * attenuation_a);
        b->angular_impulse(cross2(rb, j_f) * b->get_inv_I() * attenuation_b);
#endif /* FRICTION */
    }
}

void solve_wall_collision(RigidBody* body, const Manifold& collision) {
    assert(collision.count <= 2);

    const Vector2 n(collision.normal);
    std::array<double, 3> impulse_list;
    std::array<Vector2, 3> friction_list;
    std::array<Vector2, 3> r_list;

    for (uint8_t i(0); i < collision.count; ++i) {
        const Vector2 p(collision.contact_points[i]);

        Vector2 r(p - body->get_p());
        Vector3 r_3(r.x, r.y, 0);
        Vector3 w_3(0, 0, body->get_omega());
        Vector2 v_p(body->get_v() + Vector2(cross(w_3, r_3).x, cross(w_3, r_3).y));

        Vector2 v_r(-v_p);

        Vector2 u(triple_product(-r, r, n) * body->get_inv_I());
        double denom(body->get_inv_m() + dot2(u, n));
        double impulse(-(1 + 0.6) * dot2(v_r, n) / denom); // Walls made of wood (e = 0.6)

        impulse /= collision.count;
        // Vector2 j(n * impulse);

#ifdef FRICTION
        const double vr_n(dot2(v_r, n));
        Vector2 f_e(body->get_f());
        const double fe_n(dot2(f_e, n));
        Vector2 t;
        if (vr_n != 0) {
            t = (v_r - n * vr_n).normalized();
        }else if (fe_n != 0) {
            t = (f_e - n * fe_n).normalized();
        }

        double j_s(steel_static_friction * impulse);
        double j_d(steel_dynamic_friction * impulse);
        // double friction(body->get_mass() * dot2(v_r, t));
        double friction(-dot2(v_r, t) / (body->get_inv_m() + dot2(u, t)));

        friction /= collision.count;

        Vector2 j_t(t * friction);
        Vector2 j_f;
        if (abs(friction) <= j_s) {
            j_f = j_t;
#ifdef DEBUG
            body->set_friction_debug(true);
#endif
        }else {
            j_f = -t * j_d;
#ifdef DEBUG
            body->set_friction_debug(false);
#endif
        }
        friction_list[i] = j_f;
#endif /* FRICTION */
        impulse_list[i] = impulse;
        r_list[i] = r;
    }

    for (uint8_t i(0); i < collision.count; ++i) {
        double impulse(impulse_list[i]);
        Vector2 j(n * impulse);
        body->linear_impulse(-j * body->get_inv_m());
        Vector2 r(r_list[i]);
        body->angular_impulse(-impulse * body->get_inv_I() * cross2(r, n) * 0.5);

#ifdef FRICTION
        Vector2 j_f(friction_list[i]);
        // double attenuation(body->has_vertices() ? 0.5 : 0.7);
//// NEED TO CORRECT BOUNCING AS FRICTION INDUCES TOO MUCH ROTATION AGAINST WALLS ///////
        body->linear_impulse(-j_f * body->get_inv_m());
        body->angular_impulse(-cross2(r, j_f) * body->get_inv_I());
//////////////////////////////////////
#endif /* FRICTION */
    }
}