#ifndef COLLISION_H
#define COLLISION_H

#include <vector>
#include <SDL2/SDL.h>
#include "rigid_body.h" 


struct Cell {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    std::vector<RigidBody*> bodies;
};

struct Circle;

struct Rect {
    double x;
    double y;
    double w;
    double h;

    Rect(double x_, double y_, double w_, double h_) : x(x_), y(y_), w(w_), h(h_) {}
    bool contains(Vector2 point);
    bool intersects(Rect range);
    bool intersects(Circle range);
};

struct Circle {
    double x;
    double y;
    double r;

    Circle(double x_, double y_, double r_) : x(x_), y(y_), r(r_) {}
    bool contains(Vector2 point);
    bool intersects(Rect range);
};

class QuadTree {
    const unsigned NODE_CAPACITY = 6;

public:
    QuadTree(Rect boundary_);
    virtual ~QuadTree();

    void subdivide();
    bool insert(RigidBody* body);
    void query(Circle range, std::vector<RigidBody*>& found);

    void draw(SDL_Renderer* renderer);

private:
    Rect m_boundary;
    std::vector<RigidBody*> m_index;
    bool m_divided;

    QuadTree* north_west;
    QuadTree* north_east;
    QuadTree* south_west;
    QuadTree* south_east;
};

class CollisionGrid {
public:
    CollisionGrid();
    virtual ~CollisionGrid() {}

    void clear_cells();
    void update_cell(RigidBody* body, double x, double y);
    void solve_collisions();

protected:
    void solve_cells(Cell& cell_1, Cell& cell_2);

private:
    unsigned m_partitions;
    unsigned m_rows;
    unsigned m_cols;
    unsigned width;
    unsigned height;
    std::vector<std::vector<Cell>> m_cells;
};       

struct Result {
    Vector2 v1;
    Vector2 v2;
};

bool is_collided(Vector2 p1, double radius_1, Vector2 p2, double radius_2);
Result compute_velocity(RigidBody* body_1, RigidBody* body_2, bool is_elastic = true);

#endif /* COLLISION_H */
