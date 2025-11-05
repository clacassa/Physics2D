#include "shape.h"
#include "config.h"
#include "narrow_phase.h"
#include "render.h"
#include "transform2.h"
#include "vector2.h"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <SDL_render.h>
#include <algorithm>

namespace {
    double ccw(const Vector2 p1, const Vector2 p2, const Vector2 p3) {
        return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
    }
}

ConvexHull compute_hull(std::vector<Vector2> points) {
    ConvexHull hull;
    if (points.size() < 3) {
        return hull;
    }

    Vector2 P0(points[0]);
    std::vector<Vector2> candidates;
    candidates.push_back(P0);
    for (unsigned i(1); i < points.size(); ++i) {
        const Vector2 p(points[i]);
        if (p.y < P0.y) {
            P0 = p;
            candidates.clear();
        }else if (p.y == P0.y) {
            candidates.push_back(p);
        }
    }
    double max(INT64_MAX);
    for (auto p : candidates) {
        if (p.x < max) {
            max = p.x;
            P0 = p;
        }
    }

    // Place P0 at the front, then remove from list
    for (unsigned i(0); i < points.size(); ++i) {
        if (points[i] == P0) {
            const Vector2 temp(points[0]);
            points[0] = P0;
            points[i] = temp;
            break;
        }
    }
    points.erase(points.begin());

    std::sort(points.begin(), points.end(), [=](Vector2 a, Vector2 b)->bool {
        return dot2((a - P0).normalized(), vector2_x) > dot2((b - P0).normalized(), vector2_x);
    });

    std::vector<Vector2> stack;
    for (auto p : points) {
        while (stack.size() > 1) {
            const Vector2 next_to_top(stack[stack.size() - 2]);
            const Vector2 top(stack.back());
            if (ccw(next_to_top, top, p) > 0) {
                break;
            }
            stack.pop_back();
        }
        stack.push_back(p);
    }

    // Put back P0 to set of points
    stack.insert(stack.begin(), P0);

    const uint8_t count(stack.size());
    if (count > shape_max_vertices) {
#ifdef DEBUG
        std::cout << "The set of points resulted in a convex hull larger than 8 vertices\n";
#endif
        return hull;
    }

    for (unsigned i(0); i < count; ++i) {
        hull.points[i] = stack[i];
    }
    hull.count = count;

    return hull;
}

Shape::Shape(ConvexHull hull, double radius, ShapeType type)
:   m_type(type)
{
    if (m_type == CIRCLE) {
        m_radius = radius;
        m_count = 0;
    }else {
        // Check hull convexity and nb of vertices
        // m_vertices = ...
        // m_count = ...
        m_radius = 0;
        m_ref_vertices = hull.points;
        m_vertices = m_ref_vertices;
        m_count = hull.count;
    }
}

void Circle::transform(const Vector2 p, const double theta) {
    m_centroid = p;

    compute_aabb();
}

void Circle::translate(const Vector2 delta_p) {
    m_centroid += delta_p;
}

void Circle::rotate(const double d_theta) {
    // Stub
}

MassProperties Circle::compute_mass_properties(const double density) {
    compute_area();
    compute_centroid();

    MassProperties mp;

    mp.mass = m_area * density;
    mp.inertia = 0.5 * mp.mass * m_radius * m_radius;

    return mp;
}

bool Circle::contains_point(const Vector2 point) const {
    const Vector2 test(point - m_centroid);
    return dot2(test, test) <= m_radius * m_radius;
}

void Circle::draw(SDL_Renderer* renderer, const SDL_Color& color, bool fill) const {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

    if (fill) {
        render_circle_fill_raster(renderer, m_centroid, m_radius);
    }else {
        render_circle(renderer, m_centroid, m_radius);
    }
}

void Circle::compute_area() {
    m_area = PI * m_radius * m_radius;
}

void Circle::compute_aabb() {
    m_aabb.min = {m_centroid.x - m_radius, m_centroid.y - m_radius};
    m_aabb.max = {m_centroid.x + m_radius, m_centroid.y + m_radius};
}

void Circle::compute_centroid() {
    m_ref_centroid = vector2_zero;
    m_centroid = m_ref_centroid;
}

void Polygon::transform(const Vector2 p, const double theta) {
    const Vector2 t(p - m_ref_centroid);
    for (unsigned i(0); i < m_count; ++i) {
        m_vertices[i] = transform2(m_ref_vertices[i], t, theta, m_ref_centroid);
    }

    m_centroid = p;

    // AABB update
    compute_aabb();
}

void Polygon::translate(const Vector2 delta_p) {
    m_centroid += delta_p;
    for (uint8_t i(0); i < m_count; ++i) {
        m_vertices[i] += delta_p;
    }
}

void Polygon::rotate(const double d_theta) {
    for (uint8_t i(0); i < m_count; ++i) {
        m_vertices[i] = transform2(m_vertices[i], vector2_zero, d_theta, m_centroid);
    }
}

MassProperties Polygon::compute_mass_properties(const double density) {
    compute_area();
    compute_centroid();

    MassProperties mp;
    mp.mass = m_area * density;
    mp.inertia = 0;

    for (uint8_t i(0); i < m_count; ++i) {
        Vector2 a(m_vertices[i] - m_centroid);
        Vector2 b(m_vertices[(i + 1) % m_count] - m_centroid);
        const double mass_tri(0.5 * density * cross2(a, b));
        const double inertia_tri(mass_tri * (dot2(a, a) + dot2(b, b) + dot2(a, b)) / 6.0);
        mp.inertia += inertia_tri;
    }

    return mp;
}

bool Polygon::contains_point(const Vector2 point) const {
    // From: https://wrfranklin.org/Research/Short_Notes/pnpoly.html
    unsigned i, j = 0;
    bool c(0);
    for (i = 0, j = m_count-1; i < m_count; j = i++) {
        if ( ((m_vertices[i].y > point.y) != (m_vertices[j].y > point.y)) &&
            (point.x < (m_vertices[j].x - m_vertices[i].x) * (point.y - m_vertices[i].y) / (m_vertices[j].y - m_vertices[i].y) + m_vertices[i].x) ) {
            c = !c;
        }
    }
    return c;
}

void Polygon::draw(SDL_Renderer* renderer, const SDL_Color& color, bool fill) const {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (uint8_t i(0); i < m_count; ++i) {
        const Vector2 a(m_vertices[i]);
        const Vector2 b(m_vertices[(i + 1) % m_count]);
        render_line(renderer, a, b);
    }

    if (fill) {
        Vector2 vertices[m_count];
        for (uint8_t i(0); i < m_count; ++i) {
            vertices[i] = m_vertices[i];
        }

        uint32_t c(color.r + (color.g << 8) + (color.b << 16) + (color.a << 24));
        render_polygon_fill(renderer, vertices, m_count, c);
    }
}

void Polygon::compute_centroid() {
    double Cx(0);
    for (uint8_t i(0); i < m_count; ++i) {
        Vector2 a(m_vertices[i]);
        Vector2 b(m_vertices[(i + 1) % m_count]);
        Cx += (a.x + b.x) * cross2(a, b);
    }
    Cx /= (6 * m_area);

    double Cy(0);
    for (uint8_t i(0); i < m_count; ++i) {
        Vector2 a(m_vertices[i]);
        Vector2 b(m_vertices[(i + 1) % m_count]);
        Cy += (a.y + b.y) * cross2(a, b);
    }
    Cy /= (6 * m_area);

    m_ref_centroid = Vector2(Cx, Cy);
    m_centroid = m_ref_centroid;
}

void Polygon::compute_area() {
    double area(0);
    for (uint8_t i(0); i < m_count; ++i) {
        area += cross2(m_vertices[i], m_vertices[(i + 1) % m_count]);
    }
    m_area = 0.5 * area;
}

void Polygon::compute_aabb() {
    m_aabb.min = {support(this, -vector2_x).x, support(this, -vector2_y).y};
    m_aabb.max = {support(this, vector2_x).x, support(this, vector2_y).y};
}


Polygon create_box(const double half_width, const double half_height) {
    Vertices points;
    points[0] = Vector2(-half_width, half_height);
    points[1] = Vector2(-half_width, -half_height);
    points[2] = Vector2(half_width, -half_height);
    points[3] = Vector2(half_width, half_height);

    return Polygon(ConvexHull{points, 4});
}

Polygon create_square(const double half_side) {
    Vertices points;

    points[0] = Vector2(-half_side, half_side);
    points[1] = Vector2(-half_side, -half_side);
    points[2] = Vector2(half_side, -half_side);
    points[3] = Vector2(half_side, half_side);

    return Polygon(ConvexHull{points, 4});
}
