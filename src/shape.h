#ifndef SHAPE_H
#define SHAPE_H

#include <SDL_render.h>
#include <vector>
#include <array>
#include "vector2.h"

constexpr uint8_t shape_max_vertices(8);
typedef std::array<Vector2, shape_max_vertices> Vertices;

struct ConvexHull {
    Vertices points;
    uint8_t count = 0;
};

/**
 * @brief Computes the largest convex hull from a set of points in 2D space
 */
ConvexHull compute_hull(std::vector<Vector2> points);

enum ShapeType {
    CIRCLE,
    POLYGON
};

struct AABB {
    Vector2 min; // Bottom left corner
    Vector2 max; // Top right corner
};

struct MassProperties {
    double mass;
    double inertia;
};

class Shape {
public:
    Shape(ConvexHull hull, double radius, ShapeType type);
    virtual ~Shape() {}

    Vector2 get_centroid() const { return m_centroid; }
    Vertices get_vertices() const { return m_vertices; }
    uint8_t get_count() const { return m_count; }
    double get_radius() const { return m_radius; }
    double get_area() const { return m_area; }
    AABB get_aabb() const { return m_aabb; }
    ShapeType get_type() const { return m_type; }

    virtual void transform(const Vector2 p, const double theta) = 0;
    virtual void translate(const Vector2 delta_p) = 0;
    virtual void rotate(const double d_theta) = 0;
    virtual MassProperties compute_mass_properties(const double density) = 0;
    virtual bool contains_point(const Vector2 point) const = 0;
    virtual void draw(SDL_Renderer* renderer, const SDL_Color& color, bool fill) const = 0;
protected:
    Vector2 m_centroid;
    Vector2 m_ref_centroid;
    double m_radius;
    Vertices m_vertices;
    Vertices m_ref_vertices;
    uint8_t m_count;

    double m_area;

    AABB m_aabb;
    ShapeType m_type;

    virtual void compute_centroid() = 0;
    virtual void compute_area() = 0;
    virtual void compute_aabb() = 0;
};

class Circle : public Shape {
public:
    Circle(double radius) : Shape({}, radius, CIRCLE) {}

    void transform(const Vector2 p, const double theta) override;
    void translate(const Vector2 delta_p) override;
    void rotate(const double d_theta) override;
    MassProperties compute_mass_properties(const double density) override;
    bool contains_point(const Vector2 point) const override;
    void draw(SDL_Renderer* renderer, const SDL_Color& color, bool fill) const override;
private:
    void compute_centroid() override;
    void compute_area() override;
    void compute_aabb() override;
};

class Polygon : public Shape {
public:
    Polygon(ConvexHull hull) : Shape(hull, 0, POLYGON) {}

    void transform(const Vector2 p, const double theta) override;
    void translate(const Vector2 delta_p) override;
    void rotate(const double d_theta) override;
    MassProperties compute_mass_properties(const double density) override;
    bool contains_point(const Vector2 point) const override;
    void draw(SDL_Renderer* renderer, const SDL_Color& color, bool fill) const override;
private:
    void compute_centroid() override;
    void compute_area() override;
    void compute_aabb() override;
};

// Helper functions for polygon creation
Polygon create_box(const double half_width, const double half_height);
Polygon create_square(const double half_side);

#endif /* SHAPE_H */
