#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <array>
#include "collision.h"
#include "config.h"

using namespace std::chrono;


CollisionInfo detect_collision(RigidBody* a, RigidBody* b, double& time_1, double& time_2) {
    CollisionInfo result;
    if (!a->is_movable() && !b->is_movable())
        return result;
#ifdef SAT
    const bool a_is_polygon(a->has_vertices());
    const bool b_is_polygon(b->has_vertices());

    if (!a_is_polygon) {
        if (!b_is_polygon)
            result.intersecting = intersect_circle_circle(a, b, result);
        else {
            result.intersecting = intersect_circle_polygon(a, b, result);
        }
    }else if (!b_is_polygon) {
        result.intersecting = intersect_circle_polygon(b, a, result);
        result.normal *= -1;
    }else { // Use GJK/EPA for polygon-polygon case
# ifdef GJK_EPA
        Simplex s;
        SourcePoints points;
        auto start(steady_clock::now());
        result.intersecting = intersect_GJK(s, points, a, b);
        auto t1(steady_clock::now());

        time_1 += duration_cast<microseconds>(t1 - start).count();

        if (result.intersecting) {
            auto t2(steady_clock::now());
            EPA(s, points, a, b, result);
            auto t3(steady_clock::now());

            time_2 += duration_cast<microseconds>(t3 - t2).count();
        }
# else
        result.intersecting = intersect_polygon_polygon(a, b, result);
# endif /* GJK_EPA */
    }
#else
# ifdef GJK_EPA
    Simplex s;
    SourcePoints points;
    auto start(steady_clock::now());
    result.intersecting = intersect_GJK(s, points, a, b);
    auto t1(steady_clock::now());

    time_1 += duration_cast<microseconds>(t1 - start).count();

    if (result.intersecting) {
        auto t2(steady_clock::now());
        EPA(s, points, a, b, result);
        auto t3(steady_clock::now());

        time_2 += duration_cast<microseconds>(t3 - t2).count();
    }
# endif /* GJK_EPA */
#endif /* SAT */
    return result;
}

void solve_collision(RigidBody* a, RigidBody* b, CollisionInfo collision) {
    const Vector2 n(collision.normal);
    const Vector2 p(collision.contact_point);
    /*
     * Impulse-based reaction model
     * https://en.wikipedia.org/wiki/Collision_response
     */
    Vector2 r1(p - a->get_p());
    Vector3 r1_3(r1.x, r1.y, 0);
    Vector3 w1_3(0, 0, a->get_omega());
    Vector2 v_p1(a->get_v() + Vector2(cross(w1_3, r1_3).x, cross(w1_3, r1_3).y));

    Vector2 r2(p - b->get_p());
    Vector3 r2_3(r2.x, r2.y, 0);
    Vector3 w2_3(0, 0, b->get_omega());
    Vector2 v_p2(b->get_v() + Vector2(cross(w2_3, r2_3).x, cross(w2_3, r2_3).y));

    Vector2 v_r(v_p2 - v_p1);

    Vector2 u(triple_product(-r1, r1, n) * a->get_inv_I()
            + triple_product(-r2, r2, n) * b->get_inv_I());

    double denom(a->get_inv_m() + b->get_inv_m() + dot2(u, n));
    double impulse(-(1 + std::min(a->get_cor(), b->get_cor())) * dot2(v_r, n) / denom);
    Vector2 j(n * impulse);

    a->velocity_impulse(-j * a->get_inv_m());
    b->velocity_impulse(j * b->get_inv_m());

    // if (b->is_movable())
        a->angular_impulse(-impulse * a->get_inv_I() * cross2(r1, n));
    // if (a->is_movable())
        b->angular_impulse(impulse * b->get_inv_I() * cross2(r2, n));

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
    double friction((a->get_mass()*b->get_mass() / (a->get_mass()+b->get_mass()) * dot2(v_r, t)));
    Vector2 j_t(t * friction);
    Vector2 j_f;
    // if (dot2(v_r, t) <= 0.001)
    if (abs(friction) <= j_s) {
        j_f = -j_t;
        a->set_friction_debug(true);
        a->set_friction_debug(true);
    }else {
        j_f = -t * j_d;
        a->set_friction_debug(false);
        a->set_friction_debug(false);
    }

    a->velocity_impulse(-j_f * a->get_inv_m());
    b->velocity_impulse(j_f * b->get_inv_m());

    a->angular_impulse(-cross2(r1, j_f) * a->get_inv_I());
    b->angular_impulse(cross2(r2, j_f) * b->get_inv_I());
#endif /* FRICTION */
}

void solve_wall_collision(RigidBody* body, CollisionInfo collision) {
    const Vector2 n(collision.normal);
    const Vector2 p(collision.contact_point);
    /*
     * Impulse-based reaction model
     * https://en.wikipedia.org/wiki/Collision_response
     */
    Vector2 r(p - body->get_p());
    Vector3 r_3(r.x, r.y, 0);
    Vector3 w_3(0, 0, body->get_omega());
    Vector2 v_p(body->get_v() + Vector2(cross(w_3, r_3).x, cross(w_3, r_3).y));

    Vector2 v_r(-v_p);

    Vector2 u(triple_product(-r, r, n) * body->get_inv_I());
    double denom(body->get_inv_m() + dot2(u, n));
    double impulse(-(1 + 0.6) * dot2(v_r, n) / denom); // Walls made of wood (e = 0.6)
    Vector2 j(n * impulse);

    body->velocity_impulse(-j * body->get_inv_m());
    body->angular_impulse(-impulse * body->get_inv_I() * cross2(r, n));

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

    double j_s(0.5 * impulse);
    double j_d(0.3 * impulse);
    // double friction(body->get_mass() * dot2(v_r, t));
    double friction(-dot2(v_r, t) / (body->get_inv_m() + dot2(u, t)));
    Vector2 j_t(t * friction);
    Vector2 j_f;
    if (abs(friction) <= j_s) {
        j_f = j_t;
        body->set_friction_debug(true);
    }else {
        j_f = -t * j_d;
        body->set_friction_debug(false);
    }

//// NEED TO CORRECT ROLLING ///////
    body->velocity_impulse(-j_f * body->get_inv_m());
    // if (!body->has_vertices())
        body->angular_impulse(-cross2(r, j_f) * body->get_inv_I());
//////////////////////////////////////
#endif /* FRICTION */
}

Vector2 support(RigidBody* body, Vector2 d) {
    Vector2 support;
    if (body->has_vertices()) {
        double max(-INT_MAX);
        for (auto v : body->get_vertices()) {
            double projection(dot2(v, d));
            if (projection >= max) {
                max = projection;
                support = v;
            }
        }
    }else {
        support = body->get_p() + d.normalized() * body->get_radius();
    }

    return support;
}

/////// BROAD PHASE /////////////
std::vector<std::vector<RigidBody*>> SweepAndPrune::process(std::vector<RigidBody*>& list) {
    if (m_list.size() != list.size())
        m_list = list;
    std::vector<std::vector<RigidBody*>> possible_collisions;
    std::vector<RigidBody*> active_intervall;

    if (m_var_x >= m_var_y) {
        sort_ascending_x(m_list);
        for (unsigned i(0); i < m_list.size(); ++i) {
            for (unsigned j(0); j < active_intervall.size(); ++j) {
                if (m_list[i]->get_AABB().min.x > active_intervall[j]->get_AABB().max.x) {
                    active_intervall.erase(active_intervall.begin() +j);
                    --j;
                }else {
                    possible_collisions.push_back({m_list[i], active_intervall[j]});
                }
            }
            active_intervall.push_back(m_list[i]);
        }
    }else {
        sort_ascending_y(m_list);
        for (unsigned i(0); i < m_list.size(); ++i) {
            for (unsigned j(0); j < active_intervall.size(); ++j) {
                if (m_list[i]->get_AABB().min.y > active_intervall[j]->get_AABB().max.y) {
                    active_intervall.erase(active_intervall.begin() +j);
                    --j;
                }else {
                    possible_collisions.push_back({m_list[i], active_intervall[j]});
                }
            }
            active_intervall.push_back(m_list[i]);
        }
    }
        
    return possible_collisions;
}

void SweepAndPrune::sort_ascending_x(std::vector<RigidBody*>& list) {
    std::sort(list.begin(), list.end(), [=](RigidBody* a, RigidBody* b)->bool {
        return a->get_AABB().min.x < b->get_AABB().min.x;
    }); 
}

void SweepAndPrune::sort_ascending_y(std::vector<RigidBody*>& list) {
    std::sort(list.begin(), list.end(), [=](RigidBody* a, RigidBody* b)->bool {
        return a->get_AABB().min.y < b->get_AABB().min.y;
    }); 
}

void SweepAndPrune::choose_axis(std::vector<RigidBody*> list) {
    m_var_x = 0;
    m_var_y = 0;

    double sum_x(0), sum_y(0);
    for (auto body : list) {
        sum_x += body->get_p().x;
        sum_y += body->get_p().y;
    }
    double mean_x(sum_x / list.size());
    double mean_y(sum_y / list.size());
    for (auto body : list) {
        m_var_x += pow(body->get_p().x - mean_x, 2);
        m_var_y += pow(body->get_p().y - mean_y, 2);
    }
}

bool AABB_overlap(AABB a, AABB b) {
    double d1x(b.min.x - a.max.x);
    double d1y(b.min.y - a.max.y);
    double d2x(a.min.x - b.max.x);
    double d2y(a.min.y - b.max.y);

    return !(d1x > 0.0 || d1y > 0.0 || d2x > 0.0 || d2y > 0.0);
}


/////// NARROW PHASE ///////////
bool intersect_circle_circle(RigidBody* a, RigidBody* b, CollisionInfo& result) {
    double r_a(a->get_radius());
    double r_b(b->get_radius());
    Vector2 axis(b->get_p() - a->get_p());

    if (axis.norm() <= r_a + r_b) {
        result.normal = axis.normalized();
        result.contact_point = (a->get_p() + b->get_p()) * 0.5;
        result.depth = r_a + r_b - axis.norm();
        return true;
    }

    return false;
}

bool intersect_circle_polygon(RigidBody* a, RigidBody* b, CollisionInfo& result) {
    auto vertices(b->get_vertices());
    result.depth = INT_MAX;
    // Separation Axis Theorem
    for (size_t i(0); i < vertices.size(); ++i) {
        Vector2 A(vertices[i]);
        Vector2 B(vertices[(i + 1) % vertices.size()]);
        Vector2 edge(B - A);
        Vector2 normal(edge.normal());
        Vector2 u(a->get_p() - normal * a->get_radius() - vertices[i]);

        double support_dist(dot2(u, normal));
        if (support_dist >= 0)
            return false;

        double projection(proj2(a->get_p(), A, edge).norm());
        if (projection < edge.norm() && dot2(u, edge) > 0) {
            if (b->contains_point(u + A)) {
                result.depth = abs(support_dist);
                result.normal = -normal;
                result.contact_point = u + vertices[i] + result.normal * result.depth;
                return true;
            }
        }else {
            if (a->contains_point(A)) {
                double depth_prime(a->get_radius() - (A - a->get_p()).norm());
                if (depth_prime < result.depth) {
                    result.normal = (A - a->get_p()).normalized();
                    result.depth = depth_prime;
                    result.contact_point = A;
                }
            }else if (a->contains_point(B)) {
                double depth_prime(a->get_radius() - (B - a->get_p()).norm());
                if (depth_prime < result.depth) {
                    result.normal = (B - a->get_p()).normalized();
                    result.depth = depth_prime;
                    result.contact_point = B;
                }
            }
        }
    }

    return true;
}

bool intersect_polygon_polygon(RigidBody* a, RigidBody* b, CollisionInfo& result) {
    auto a_vertices(a->get_vertices());
    auto b_vertices(b->get_vertices());
    result.depth = INT_MAX;
    // Separation Axis Theorem
    for (size_t i(0); i < a_vertices.size(); ++i) {
        Vector2 edge(a_vertices[(i + 1) % a_vertices.size()] - a_vertices[i]);
        Vector2 normal(edge.normal());
        bool all_in_front(true);
        double current_depth(0);
        for (auto vertex : b_vertices) {
            double projection(dot2(vertex - a_vertices[i], normal));
            if (projection <= 0) {
                all_in_front = false;
                if (abs(projection) > current_depth && a->contains_point(vertex)) {
                    current_depth = abs(projection);
                    if (current_depth < result.depth) {
                        result.depth = current_depth;
                        result.normal = normal;
                        result.contact_point = vertex + result.normal * result.depth;
                    }
                }
            }
        }
        if (all_in_front)
            return false;
    }
    double depth_2(1e6);
    Vector2 n_2;
    Vector2 p_2;
    for (size_t i(0); i < b_vertices.size(); ++i) {
        Vector2 edge(b_vertices[(i + 1) % b_vertices.size()] - b_vertices[i]);
        Vector2 normal(edge.normal());
        bool all_in_front(true);
        double current_depth(0);
        for (auto vertex : a_vertices) {
            double projection(dot2(vertex - b_vertices[i], normal));
            if (projection <= 0) {
                all_in_front = false;
                if (abs(projection) > current_depth && b->contains_point(vertex)) {
                    current_depth = abs(projection);
                    if (current_depth < depth_2) {
                        depth_2 = current_depth;
                        n_2 = normal * -1;
                        p_2 = vertex + n_2 * depth_2;
                    }
                }
            }
        }
        if (all_in_front)
            return false;
    }

    if ((depth_2 >= result.depth && depth_2 != INT_MAX) || result.depth == INT_MAX) {
        result.normal = n_2;
        result.contact_point = p_2;
        result.depth = depth_2;
    }
    return true;
}


bool intersect_GJK(Simplex& s, SourcePoints& shape_points, RigidBody* a, RigidBody* b) {
    const Vector2 axis(1, 0); // Arbitrary initial axis
    Vector2 S(support(a, axis) - support(b, -axis));
    Vector2 D(-S);

    size_t watchdog(0);
    while (true) {
        Vector2 supp_a(support(a, D));
        Vector2 supp_b(support(b, -D));
        Vector2 A(supp_a - supp_b);
        if (dot2(A, D) < 0)
            return false;
        s.push_back(A);
        shape_points.push_back({supp_a, supp_b});
        if (nearest_simplex(s, D, shape_points))
            return true;

        ++watchdog;
        if (watchdog > 1e5) {
            std::cout << "GJK INFINITE!\n";
            exit(1);
        }
    }
}

bool nearest_simplex(Simplex& s, Vector2& D, SourcePoints& shape_points) {
    const size_t n(s.size());
    if (n == 1) {
        Vector2 A(s[0]);
        D = -A;
    }else if (n == 2) {
        const Vector2 A(s[1]);
        const Vector2 B(s[0]);
        const std::array<Vector2, 2> source_B(shape_points[0]);
        if (dot2(B - A, -A) > 0) {
            s[0] = A;
            s[1] = B;
            shape_points[0] = shape_points[1];
            shape_points[1] = source_B;
            D = triple_product(A - B, B - A, -A);
        }else {
            s[0] = A;
            s.pop_back();
            shape_points[0] = shape_points[1];
            shape_points.pop_back();
            D = -A;
        }
    }else if (n == 3) {
        // Test if the simplex (triangle) contains the origin.
        // If not, compute the closest simplex to the origin.
        double cp1(cross2(s[1] - s[0], -s[0]));
        double cp2(cross2(s[2] - s[1], -s[1]));
        double cp3(cross2(s[0] - s[2], -s[2]));
        if (!((cp1 < 0 || cp2 < 0 || cp3 < 0) && (cp1 >= 0 || cp2 >= 0 || cp3 >= 0)))
            return true;
        else {
            Vector2 A(s[2]);
            Vector2 B(s[1]);
            Vector2 C(s[0]);
            const std::array<Vector2, 2> source_B(shape_points[1]);
            const std::array<Vector2, 2> source_C(shape_points[0]);
            Vector2 AB(B - A);
            Vector2 BC(C - B);
            Vector2 AC(C - A);
            if (dot2(triple_product(-AC, AB, AC), -A) > 0) {
                if (dot2(AC, -A) > 0) {
                    s[0] = A;
                    s[1] = C;
                    s.pop_back();
                    shape_points[0] = shape_points[2];
                    shape_points[1] = source_C;
                    shape_points.pop_back();
                    D = triple_product(AC, -A, AC);
                }else {
                    if (dot2(AB, -A) > 0) {
                        s[0] = A;
                        s[1] = B;
                        s.pop_back();
                        shape_points[0] = shape_points[2];
                        shape_points.pop_back();
                        D = triple_product(AB, -A, AB);
                    }else {
                        s[0] = A;
                        s.pop_back();
                        s.pop_back();
                        shape_points[0] = shape_points[2];
                        shape_points.pop_back();
                        shape_points.pop_back();
                        D = -A;
                    }
                }
            }else {
                if (dot2(triple_product(AB, AB, AC), -A) > 0) {
                    if (dot2(AB, -A) > 0) {
                        s[0] = A;
                        s[1] = B;
                        s.pop_back();
                        shape_points[0] = shape_points[2];
                        shape_points.pop_back();
                        D = triple_product(AB, -A, AB);
                    }else {
                        s[0] = A;
                        s.pop_back();
                        s.pop_back();
                        shape_points[0] = shape_points[2];
                        shape_points.pop_back();
                        shape_points.pop_back();
                        D = -A;
                    }
                }else {
                    return true;
                }
            }
        }
    }
    return false;
}

void EPA(Simplex s, SourcePoints points, RigidBody* a, RigidBody* b, CollisionInfo& result) {
    // Determine the winding of the simplex
    double winding(0);
    for (size_t i(0); i < s.size() - 1; ++i) {
        winding += s[i].x * s[i + 1].y - s[i + 1].x * s[i].y;
    }
    const bool clockwise(winding < 0);

    size_t watchdog(0);
    while (true) {
        Edge e(closest_edge(s, clockwise));
        Vector2 supp_a(support(a, e.normal));
        Vector2 supp_b(support(b, -e.normal));
        Vector2 supp(supp_a - supp_b);
        double d(dot2(supp, e.normal));
        if (d - e.distance < EPA_epsilon) {
            result.normal = e.normal;
            result.depth = d;
            Vector2 MTV(-e.normal * d);
            result.contact_point = convex_combination(s,points,e.index).closest_a;
            a->set_test(result.contact_point);
            break;
        }else {
            s.insert(s.begin() + e.index, supp);
            points.insert(points.begin() + e.index, {supp_a, supp_b});
        }
        ++watchdog;
        if (watchdog > 10000) {
            std::cout << "INFINITE LOOP\n";
            exit(1);
        }
    }
}

Edge closest_edge(Simplex s, const bool clockwise) {
    Edge closest;
    closest.distance = INT_MAX;

    for (size_t i(0); i < s.size(); i++) {
        size_t j((i + 1) == s.size() ? 0 : i + 1);
        Vector2 A(s[i]);
        Vector2 B(s[j]);
        Vector2 edge(B - A);
        // Vector2 ABO(triple_product(edge * -1, edge, A).normalized());
        Vector2 ABO;
        if (clockwise) {
            ABO = {edge.y, -edge.x};
        }else {
            ABO = {-edge.y, edge.x};
        }
        ABO = ABO.normalized();
        // Distance from the origin to the edge
        double d(dot2(ABO, A));
        // Check the distance against the other distances
        if (d < closest.distance) {
            closest.distance = d;
            closest.normal = ABO;
            closest.index = j;
        }
    }

    return closest;
}

ClosestPoints convex_combination(Simplex s, SourcePoints shape_points, size_t index) {
    ClosestPoints points;

    Vector2 p1_a(shape_points[index == 0 ? s.size() - 1 : index - 1][0]);
    Vector2 p2_a(shape_points[index][0]);
    Vector2 p1_b(shape_points[index == 0 ? s.size() - 1 : index - 1][1]);
    Vector2 p2_b(shape_points[index][1]);

    Vector2 l(s[index] - s[index == 0 ? s.size() - 1 : index - 1]);
    if (l == Vector2(0, 0)) {
        points.closest_a = p1_a;
        points.closest_b = p1_b;
    }else {
        double lambda_2(-dot2(s[index == 0 ? s.size() - 1 : index - 1], l) / dot2(l, l));
        double lambda_1(1 - lambda_2);
        if (lambda_1 < 0) {
            points.closest_a = p2_a;
            points.closest_b = p2_b;
        } else if (lambda_2 < 0) {
            points.closest_a = p1_a;
            points.closest_b = p1_b;
        }else {
            points.closest_a = p1_a * lambda_1 + p2_a * lambda_2;
            points.closest_b = p1_b * lambda_1 + p2_b * lambda_2;
        }
    }

    return points;
}

// void solve_circle_circle(RigidBody* a, RigidBody* b, bool is_elastic) {
//     Vector2 v1(a->get_v());
//     Vector2 v2(b->get_v());
//     Vector2 p1(a->get_p());
//     Vector2 p2(b->get_p());
//     double m1(a->get_mass());
//     double m2(b->get_mass());

//     Vector2 v1_new;
//     Vector2 v2_new;
//     Vector2 d_v1({v1.x - v2.x, v1.y - v2.y});
//     Vector2 d_v2({v2.x - v1.x, v2.y - v1.y});
//     Vector2 d_p1({p1.x - p2.x, p1.y - p2.y});
//     Vector2 d_p2({p2.x - p1.x, p2.y - p1.y});
//     if (is_elastic) {
//         v1_new = (v1 - d_p1 * 2 * m2 / (m1 + m2) * dot2(d_v1, d_p1) / pow(d_p1.norm(), 2));
//         v2_new = (v2 - d_p2 * 2 * m1 / (m1 + m2) * dot2(d_v2, d_p2) / pow(d_p2.norm(), 2));

//         // a->p += d_p1.normalized() * 0.5 * (a->r + b->r - d_p1.norm());
//         // b->p += d_p2.normalized() * 0.5 * (a->r + b->r - d_p1.norm());
//     }else {
//         v1_new.x = (m1 * v1.x + m2 * v2.x) / (m1 + m2);
//         v1_new.y = (m1 * v1.y + m2 * v2.y) / (m1 + m2);

//         v2_new = v1_new;
//     }
// }



// bool Rect::contains(Vector2 point) {
//     return (point.x > x - w && point.x <= x + w && point.y > y - h && point.x < y + h);
// }

// bool Rect::intersects(Rect range) {
//     return !(range.x - range.w > x + w || range.x + w < x - w
//             || range.y - range.h > y + h || range.y + h < y - h);
// }

// bool Rect::intersects(Circle range) {
//     double x_dist(abs(range.x - x));
//     double y_dist(abs(range.y - y));
    
//     double edges(pow(x_dist - w, 2) + pow(y_dist - h, 2));

//     if (x_dist > range.r + w || y_dist > range.r + h)
//         return false;
//     if (x_dist <= w || y_dist <= h)
//         return true;
//     return edges <= range.r * range.r;
// }

// bool Circle::contains(Vector2 point) {
//     Vector2 v(point.x - x, point.y - y);
//     return (v.norm() * v.norm() <= r * r);
// }

// bool Circle::intersects(Rect range) {
//     double x_dist(abs(range.x - x));
//     double y_dist(abs(range.y - y));
//     double w(range.w);
//     double h(range.h);

//     double edges(pow(x_dist - w, 2) + pow(y_dist - h, 2));

//     if (x_dist > r + w || y_dist > r +h)
//         return false;
//     if (x_dist <= w || y_dist <= h)
//         return true;
//     return edges <= r * r;
// }

// QuadTree::QuadTree(Rect boundary_)
// :   m_boundary(boundary_),
//     m_divided(false),
//     north_west(nullptr),
//     north_east(nullptr),
//     south_west(nullptr),
//     south_east(nullptr)
// {}

// QuadTree::~QuadTree() {
//     delete north_west;
//     delete north_east;
//     delete south_west;
//     delete south_east;
// }

// void QuadTree::subdivide() {
//     double x(m_boundary.x);
//     double y(m_boundary.y);
//     double w(m_boundary.w);
//     double h(m_boundary.h);

//     Rect nw(x - w / 2, y - h / 2, w / 2, h / 2);
//     north_west = new QuadTree(nw);
//     Rect ne(x + w / 2, y - h / 2, w / 2, h / 2);
//     north_east = new QuadTree(ne);
//     Rect sw(x - w / 2, y + h / 2, w / 2, h / 2);
//     south_west = new QuadTree(sw);
//     Rect se(x + w / 2, y + h / 2, w / 2, h / 2);
//     south_east = new QuadTree(se);

//     /*for (auto& body : m_index) {
//         if (north_west->insert(body))
//             continue;
//         if (north_east->insert(body))
//             continue;
//         if (south_west->insert(body))
//             continue;
//         if (south_east->insert(body))
//             continue;
//     }*/

//     //m_index.clear();
//     m_divided = true;
// }

// bool QuadTree::insert(RigidBody* body) {
//     if (!m_boundary.contains(body->get_p()))
//         return false;

//     if (m_index.size() < NODE_CAPACITY && !m_divided) {
//         m_index.push_back(body);
//         return true;
//     }

//     if (!m_divided)
//         subdivide();

//     if (north_west->insert(body))
//         return true;
//     if (north_east->insert(body))
//         return true;
//     if (south_west->insert(body))
//         return true;
//     if (south_east->insert(body))
//         return true;

//     return false;
// }

// void QuadTree::query(Circle range, std::vector<RigidBody*>& found) {
//     if (m_boundary.intersects(range)) {
//         for (auto& body : m_index) {
//             if (range.contains(body->get_p()))
//                 found.emplace_back(body);
//         }

//         if (m_divided) {
//             north_west->query(range, found);
//             //found.insert(found.end(), vec.begin(), vec.end());
//             north_east->query(range, found);
//             //found.insert(found.end(), vec.begin(), vec.end());
//             south_west->query(range, found);
//             //found.insert(found.end(), vec.begin(), vec.end());
//             south_east->query(range, found);
//             //found.insert(found.end(), vec.begin(), vec.end());
//         }
//     }
// }

// void QuadTree::draw(SDL_Renderer* renderer) {
//     if (m_divided) {
//         double x(m_boundary.x);
//         double y(m_boundary.y);
//         double w(m_boundary.w);
//         double h(m_boundary.h);

//         SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
//         SDL_RenderDrawLine(renderer, x, y - h, x, y + h);
//         SDL_RenderDrawLine(renderer, x - w, y, x + w, y);

//         if (north_west)
//             north_west->draw(renderer);
//         if (north_east)
//             north_east->draw(renderer);
//         if (south_west)
//             south_west->draw(renderer);
//         if (south_east)
//             south_east->draw(renderer);
//     }
// }


// CollisionGrid::CollisionGrid()
// :   m_partitions(25), // Must be a power of 2 
//     //m_rows(m_partitions / (log2(m_partitions) + 1)),
//     //m_cols(m_partitions / (log2(m_partitions) - 1)),
//     m_rows(sqrt(m_partitions)),
//     m_cols(sqrt(m_partitions)),
//     m_cells(m_rows, std::vector<Cell>(m_cols)) 
// {
//     // for (unsigned i(0); i < m_rows; ++i) {
//     //     for (unsigned j(0); j < m_cols; ++j) {
//     //         m_cells[i][j].x_min = (j / (double)m_cols) * SCENE_WIDTH;
//     //         m_cells[i][j].x_max = ((j + 1) / (double)m_cols) * SCENE_WIDTH;
//     //         m_cells[i][j].y_min = ((m_rows - (i + 1)) / (double)m_rows) * SCENE_HEIGHT;
//     //         m_cells[i][j].y_max = ((m_rows - i) / (double)m_rows) * SCENE_HEIGHT;
//     //     }
//     // }

//     // std::cout << m_rows << " " << m_cols << "\n";
// }

// void CollisionGrid::clear_cells() {
//     for (auto& row : m_cells) {
//         for (auto& cell : row) {
//             cell.bodies.clear();
//         }
//     }
// }

// void CollisionGrid::update_cell(RigidBody* body, double x, double y) {
//     for (auto& row : m_cells) {
//         for (auto& cell : row) {
//             if (x > cell.x_min && x <= cell.x_max && y > cell.y_min && y <= cell.y_max) {
//                 cell.bodies.push_back(body);
//             }
//         }
//     }
// }

// void CollisionGrid::solve_collisions() {
//     for (unsigned x(0); x < m_cols; ++x) {
//         for (unsigned y(0); y < m_rows; ++y) {
//             Cell& cell(m_cells[y][x]);
//             for (int dx(-1); dx <= 1; ++dx) {
//                 for (int dy(-1); dy <= 1; ++dy) {
//                     if (x + dx < 0 || x + dx >= m_cols || y + dx < 0 || y + dy >= m_rows)
//                         continue;
//                     Cell& other_cell(m_cells[y + dy][x + dx]);
//                     solve_cells(cell, other_cell);
//                 }
//             }
//         }
//     } 
// }

// void CollisionGrid::solve_cells(Cell& cell_1, Cell& cell_2) {
//     for (auto& body_1 : cell_1.bodies) {
//         for (auto& body_2 : cell_2.bodies) {
//             if (body_1 != body_2) {
//                 // if (is_collided(body_1->p, body_1->r, body_2->p, body_2->r)) {
//                 //     Result result(compute_velocity(body_1, body_2));
//                 //     body_1->v = result.v1;
//                 //     body_2->v = result.v2;
//                 // }
//             }
//         }
//     }
// }

