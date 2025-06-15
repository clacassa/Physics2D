#include "shape.h"
#include "config.h"
#include "narrow_phase.h"
#include "transform2.h"
#include "vector2.h"
#include <cassert>
#include <iostream>

Shape::Shape(Vertices points, double radius, ShapeType type)
:   m_type(type)
{
    if (m_type == CIRCLE) {
        m_radius = radius;
        m_count = 0;
        m_vertices.clear();
    }else {
        // Check hull convexity and nb of vertices
        // m_vertices = ...
        // m_count = ...
        // m_radius = ...
        assert(points.size() <= shape_max_vertices);
        m_ref_vertices = points;
        m_vertices = m_ref_vertices;
        m_count = m_ref_vertices.size();
    }
}

Circle::Circle(double radius) : Shape({}, radius, CIRCLE) {
    compute_area();
    compute_centroid();
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

MassProperties Circle::compute_mass_properties(const double density) const {
    MassProperties mp;

    mp.mass = m_area * density;
    mp.inertia = 0.5 * mp.mass * m_radius * m_radius;

    return mp;
}

bool Circle::contains_point(const Vector2 point) const {
    const Vector2 test(point - m_centroid);
    return dot2(test, test) <= m_radius * m_radius;
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


Polygon::Polygon(Vertices vertices) : Shape(vertices, 0, POLYGON) {
    compute_area();
    compute_centroid();
}

void Polygon::transform(const Vector2 p, const double theta) {
    const Vector2 t(p - m_ref_centroid);
    for (unsigned i(0); i < m_count; ++i) {
        m_vertices[i] = transform2(m_ref_vertices[i], t, theta, vector2_zero);
    }

    m_centroid = p;

    // AABB update
    compute_aabb();
}

void Polygon::translate(const Vector2 delta_p) {
    m_centroid += delta_p;
    for (auto& v : m_vertices) {
        v += delta_p;
    }
}

void Polygon::rotate(const double d_theta) {
    for (auto& v : m_vertices) {
        v = transform2(v, vector2_zero, d_theta, m_centroid);
    }
}

MassProperties Polygon::compute_mass_properties(const double density) const {
    MassProperties mp;
    mp.mass = m_area * density;
    mp.inertia = 0;

    for (unsigned i(0); i < m_count; ++i) {
        Vector2 a(m_vertices[i] - m_centroid);
        Vector2 b(m_vertices[(i + 1) % m_count] - m_centroid);
        const double mass_tri(0.5 * density * cross2(a, b));
        const double inertia_tri(mass_tri * (dot2(a, a) + dot2(b, b) + dot2(a, b)) / 6.0);
        mp.inertia += inertia_tri;
    }

    return mp;
}

bool Polygon::contains_point(const Vector2 point) const {
    Vector2 AP(point - m_vertices[0]);
    Vector2 AB(m_vertices[1] - m_vertices[0]);
    Vector2 AD(m_vertices[3] - m_vertices[0]);

    return (0 <= dot2(AP, AB) && dot2(AP, AB) <= dot2(AB, AB)
         && 0 <= dot2(AP, AD) && dot2(AP, AD) <= dot2(AD, AD));
}

void Polygon::compute_centroid() {
    double Cx(0);
    for (unsigned i(0); i < m_count; ++i) {
        Vector2 a(m_vertices[i]);
        Vector2 b(m_vertices[(i + 1) % m_count]);
        Cx += (a.x + b.x) * cross2(a, b);
    }
    Cx /= (6 * m_area);

    double Cy(0);
    for (unsigned i(0); i < m_count; ++i) {
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
    for (unsigned i(0); i < m_count; ++i) {
        area += cross2(m_vertices[i], m_vertices[(i + 1) % m_count]);
    }
    m_area = 0.5 * area;
}

void Polygon::compute_aabb() {
    m_aabb.min = {support(this, -vector2_x).x, support(this, -vector2_y).y};
    m_aabb.max = {support(this, vector2_x).x, support(this, vector2_y).y};
}


Shape* create_circle(const double radius) {
    return new Circle(radius);
}

Shape* create_box(const double half_width, const double half_height) {
    Vertices vertices;
    vertices.push_back(Vector2(-half_width, half_height));
    vertices.push_back(Vector2(-half_width, -half_height));
    vertices.push_back(Vector2(half_width, -half_height));
    vertices.push_back(Vector2(half_width, half_height));

    return new Polygon(vertices);
}

