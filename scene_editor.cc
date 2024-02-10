#include <iostream>
#include "scene_editor.h"

SceneEditor::SceneEditor(double division)
:   n(SCENE_HEIGHT / division),
    active_node(0, 0) 
{
    m_grid.resize(n, std::vector<Vector2>(n * 2));
    for (size_t i(0); i < n; ++i) {
        for (size_t j(0); j < n * 2; ++j) {
            m_grid[i][j] = Vector2(j * division, i * division);
        }
    }
}

void SceneEditor::render_grid(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 128);
    for (auto row : m_grid) {
        for (auto node : row) {
            render_point(renderer, node.x, node.y);
            // render_line(renderer, 0, 0, node.x, node.y);
        }
    }
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    double x(active_node.x);
    double y(active_node.y);
    render_line(renderer, x - 0.1, y, x + 0.1, y);
    render_line(renderer, x, y - 0.1, x, y + 0.1);
}

Vector2 SceneEditor::track_point(Vector2 p) {
    Vector2 tracked_point;
    double dist(INT_MAX);
    for (size_t i(0); i < n; ++i) {
        for (size_t j(0); j < n * 2; ++j) {
            const Vector2 node(m_grid[i][j]);
            const Vector2 v(node - p);
            double d(v.x * v.x + v.y * v.y);
            if (d < dist) {
                tracked_point = node;
                dist = d;
            }
        }
    }
    active_node = tracked_point;
    return tracked_point;
}