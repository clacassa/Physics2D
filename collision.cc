#include <iostream>
#include <cmath>
#include "collision.h"
#include "render.h"


bool Rect::contains(Vector2 point) {
    return (point.x > x - w && point.x <= x + w && point.y > y - h && point.x < y + h);
}

bool Rect::intersects(Rect range) {
    return !(range.x - range.w > x + w || range.x + w < x - w
            || range.y - range.h > y + h || range.y + h < y - h);
}

bool Rect::intersects(Circle range) {
    double x_dist(abs(range.x - x));
    double y_dist(abs(range.y - y));
    
    double edges(pow(x_dist - w, 2) + pow(y_dist - h, 2));

    if (x_dist > range.r + w || y_dist > range.r + h)
        return false;
    if (x_dist <= w || y_dist <= h)
        return true;
    return edges <= range.r * range.r;
}

bool Circle::contains(Vector2 point) {
    Vector2 v(point.x - x, point.y - y);
    return (v.norm() * v.norm() <= r * r);
}

bool Circle::intersects(Rect range) {
    double x_dist(abs(range.x - x));
    double y_dist(abs(range.y - y));
    double w(range.w);
    double h(range.h);

    double edges(pow(x_dist - w, 2) + pow(y_dist - h, 2));

    if (x_dist > r + w || y_dist > r +h)
        return false;
    if (x_dist <= w || y_dist <= h)
        return true;
    return edges <= r * r;
}

QuadTree::QuadTree(Rect boundary_)
:   m_boundary(boundary_),
    m_divided(false),
    north_west(nullptr),
    north_east(nullptr),
    south_west(nullptr),
    south_east(nullptr)
{}

QuadTree::~QuadTree() {
    delete north_west;
    delete north_east;
    delete south_west;
    delete south_east;
}

void QuadTree::subdivide() {
    double x(m_boundary.x);
    double y(m_boundary.y);
    double w(m_boundary.w);
    double h(m_boundary.h);

    Rect nw(x - w / 2, y - h / 2, w / 2, h / 2);
    north_west = new QuadTree(nw);
    Rect ne(x + w / 2, y - h / 2, w / 2, h / 2);
    north_east = new QuadTree(ne);
    Rect sw(x - w / 2, y + h / 2, w / 2, h / 2);
    south_west = new QuadTree(sw);
    Rect se(x + w / 2, y + h / 2, w / 2, h / 2);
    south_east = new QuadTree(se);

    /*for (auto& body : m_index) {
        if (north_west->insert(body))
            continue;
        if (north_east->insert(body))
            continue;
        if (south_west->insert(body))
            continue;
        if (south_east->insert(body))
            continue;
    }*/

    //m_index.clear();
    m_divided = true;
}

bool QuadTree::insert(RigidBody* body) {
    if (!m_boundary.contains(body->p))
        return false;

    if (m_index.size() < NODE_CAPACITY && !m_divided) {
        m_index.push_back(body);
        return true;
    }

    if (!m_divided)
        subdivide();

    if (north_west->insert(body))
        return true;
    if (north_east->insert(body))
        return true;
    if (south_west->insert(body))
        return true;
    if (south_east->insert(body))
        return true;

    return false;
}

void QuadTree::query(
        Circle range,
        std::vector<RigidBody*>& found)
{
    if (m_boundary.intersects(range)) {
        for (auto& body : m_index) {
            if (range.contains(body->p))
                found.emplace_back(body);
        }

        if (m_divided) {
            north_west->query(range, found);
            //found.insert(found.end(), vec.begin(), vec.end());
            north_east->query(range, found);
            //found.insert(found.end(), vec.begin(), vec.end());
            south_west->query(range, found);
            //found.insert(found.end(), vec.begin(), vec.end());
            south_east->query(range, found);
            //found.insert(found.end(), vec.begin(), vec.end());
        }
    }
}

void QuadTree::draw(SDL_Renderer* renderer) {
    if (m_divided) {
        double x(m_boundary.x);
        double y(m_boundary.y);
        double w(m_boundary.w);
        double h(m_boundary.h);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderDrawLine(renderer, x, y - h, x, y + h);
        SDL_RenderDrawLine(renderer, x - w, y, x + w, y);

        if (north_west)
            north_west->draw(renderer);
        if (north_east)
            north_east->draw(renderer);
        if (south_west)
            south_west->draw(renderer);
        if (south_east)
            south_east->draw(renderer);
    }
}


CollisionGrid::CollisionGrid()
:   m_partitions(25), // Must be a power of 2 
    //m_rows(m_partitions / (log2(m_partitions) + 1)),
    //m_cols(m_partitions / (log2(m_partitions) - 1)),
    m_rows(sqrt(m_partitions)),
    m_cols(sqrt(m_partitions)),
    m_cells(m_rows, std::vector<Cell>(m_cols)) 
{
    for (unsigned i(0); i < m_rows; ++i) {
        for (unsigned j(0); j < m_cols; ++j) {
            m_cells[i][j].x_min = (j / (double)m_cols) * SCENE_WIDTH;
            m_cells[i][j].x_max = ((j + 1) / (double)m_cols) * SCENE_WIDTH;
            m_cells[i][j].y_min = ((m_rows - (i + 1)) / (double)m_rows) * SCENE_HEIGHT;
            m_cells[i][j].y_max = ((m_rows - i) / (double)m_rows) * SCENE_HEIGHT;
        }
    }

    std::cout << m_rows << " " << m_cols << "\n";
}

void CollisionGrid::clear_cells() {
    for (auto& row : m_cells) {
        for (auto& cell : row) {
            cell.bodies.clear();
        }
    }
}

void CollisionGrid::update_cell(RigidBody* body, double x, double y) {
    for (auto& row : m_cells) {
        for (auto& cell : row) {
            if (x > cell.x_min && x <= cell.x_max && y > cell.y_min && y <= cell.y_max) {
                cell.bodies.push_back(body);
            }
        }
    }
}

void CollisionGrid::solve_collisions() {
    for (unsigned x(0); x < m_cols; ++x) {
        for (unsigned y(0); y < m_rows; ++y) {
            Cell& cell(m_cells[y][x]);
            for (int dx(-1); dx <= 1; ++dx) {
                for (int dy(-1); dy <= 1; ++dy) {
                    if (x + dx < 0 || x + dx >= m_cols || y + dx < 0 || y + dy >= m_rows)
                        continue;
                    Cell& other_cell(m_cells[y + dy][x + dx]);
                    solve_cells(cell, other_cell);
                }
            }
        }
    } 
}

void CollisionGrid::solve_cells(Cell& cell_1, Cell& cell_2) {
    for (auto& body_1 : cell_1.bodies) {
        for (auto& body_2 : cell_2.bodies) {
            if (body_1 != body_2) {
                // if (is_collided(body_1->p, body_1->r, body_2->p, body_2->r)) {
                //     Result result(compute_velocity(body_1, body_2));
                //     body_1->v = result.v1;
                //     body_2->v = result.v2;
                // }
            }
        }
    }
}

bool is_collided(Vector2 p1, double radius_1, Vector2 p2, double radius_2) {
    return (Vector2{p1.x - p2.x, p1.y - p2.y}.norm() <= radius_1 + radius_2);
}     

Result compute_velocity(RigidBody* body_1, RigidBody* body_2, bool is_elastic) {
    Vector2 v1(body_1->v);
    Vector2 v2(body_2->v);
    Vector2 p1(body_1->p);
    Vector2 p2(body_2->p);
    double m1(body_1->m);
    double m2(body_2->m);

    Vector2 v1_new;
    Vector2 v2_new;
    Vector2 d_v1({v1.x - v2.x, v1.y - v2.y});
    Vector2 d_v2({v2.x - v1.x, v2.y - v1.y});
    Vector2 d_p1({p1.x - p2.x, p1.y - p2.y});
    Vector2 d_p2({p2.x - p1.x, p2.y - p1.y});
    if (is_elastic) {
        v1_new = (v1 - d_p1 * 2 * m2 / (m1 + m2) * dot2(d_v1, d_p1) / pow(d_p1.norm(), 2));
        v2_new = (v2 - d_p2 * 2 * m1 / (m1 + m2) * dot2(d_v2, d_p2) / pow(d_p2.norm(), 2));

        body_1->p += d_p1.normalized() * 0.5 * (body_1->r + body_2->r - d_p1.norm());
        body_2->p += d_p2.normalized() * 0.5 * (body_1->r + body_2->r - d_p1.norm());
    }else {
        v1_new.x = (m1 * v1.x + m2 * v2.x) / (m1 + m2);
        v1_new.y = (m1 * v1.y + m2 * v2.y) / (m1 + m2);

        v2_new = v1_new;
    }
    return Result{v1_new, v2_new};
}

