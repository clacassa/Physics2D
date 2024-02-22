#include <iostream>
#include <climits>
#include "editor.h"

Editor::Editor(SDL_Renderer* renderer, TTF_Font* font, double division)
:   div(division),
    n(SCENE_HEIGHT / div + 1),
    active_node(0, 0),
    m_renderer(renderer),
    m_font(font),
    m_title(m_renderer, text_color, m_font),
    m_banner(m_renderer, 10), // centered layout
    movable_body(true),
    enabled_body(true),
    show_help_banner(true),
    very_stiff_spring(false),
    body_type(BALL),
    damping(Spring::UNDERDAMPED)
{
    m_grid.resize(n, std::vector<Vector2>(n * 2));
    for (size_t i(0); i < n; ++i) {
        for (size_t j(0); j < n * 2; ++j) {
            m_grid[i][j] = Vector2(j * div, i * div);
        }
    }

    m_banner.add_texture(text_color, m_font);
    m_banner.add_texture(text_color, m_font);
    update_help();
}

void Editor::update_help() {
    help = "<: Smaller grid       >: Larger grid       LMB: Add a body       ";
    help += "B: Ball       ";
    if (body_type == BALL)
        help.replace(help.length() - 6, 1, "*");
    help += "R: Rectangle       ";
    if (body_type == RECTANGLE)
        help.replace(help.length() - 6, 1, "*");
    help += "9: Movable body [  ]     ";
    if (movable_body)
        help.replace(help.length() - 9, 4, "[v/]");
    help += "0: Enabled body [  ]     ";
    if (enabled_body)
        help.replace(help.length() - 9, 4, "[v/]");
    help += "\n\nRMB: Add a spring       4: Elastic [  ]     ";
    if (!very_stiff_spring)
        help.replace(help.length() - 9, 4, "[v/]");
    help += "Damping options:  5: Undamped       ";
    if (damping == Spring::UNDAMPED)
        help.replace(help.length() - 6, 1, "*");
    help += "6: Underdamped       ";
    if (damping == Spring::UNDERDAMPED)
        help.replace(help.length() - 6, 1, "*");
    help += "7: Critically damped       ";
    if (damping == Spring::CRIT_DAMPED)
        help.replace(help.length() - 6, 1, "*");
    help += "8: Overdamped       ";
    if (damping == Spring::OVERDAMPED)
        help.replace(help.length() - 6, 1, "*");
}

void Editor::render_grid() {
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
    for (auto row : m_grid) {
        for (auto node : row) {
            render_point(m_renderer, node.x, node.y);
        }
    }
    // Render the cross pointer
    SDL_SetRenderDrawColor(m_renderer, 255, 0, 255, 255);
    double x(active_node.x);
    double y(active_node.y);
    render_line(m_renderer, x - 10 / RENDER_SCALE, y, x + 10 / RENDER_SCALE, y);
    render_line(m_renderer, x, y - 10 / RENDER_SCALE, x, y + 10 / RENDER_SCALE);
    // Render the median axis with graduations
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 128);
    render_line(m_renderer, 0, SCENE_HEIGHT * 0.5, SCENE_WIDTH, SCENE_HEIGHT * 0.5);
    render_line(m_renderer, SCENE_WIDTH * 0.5, 0, SCENE_WIDTH * 0.5, SCENE_HEIGHT);
    double p1, p2;
    for (auto row : m_grid) {
        if (row[0].y - floor(row[0].y) == 0) {
            p1 = SCENE_WIDTH * 0.5 - 7.5 / RENDER_SCALE;
            p2 = SCENE_WIDTH * 0.5 + 7.5 / RENDER_SCALE;
        }else {
            p1 = SCENE_WIDTH * 0.5 - 2.5 / RENDER_SCALE;
            p2 = SCENE_WIDTH * 0.5 - 2.5 / RENDER_SCALE;
        }
        render_line(m_renderer, p1, row[0].y, p2, row[0].y);
    }
    for (auto node : m_grid[0]) {
        if (node.x - floor(node.x) == 0) {
            p1 = SCENE_HEIGHT * 0.5 - 7.5 / RENDER_SCALE;
            p2 = SCENE_HEIGHT * 0.5 + 7.5 / RENDER_SCALE;
        }else {
            p1 = SCENE_HEIGHT * 0.5 - 2.5 / RENDER_SCALE;
            p2 = SCENE_HEIGHT * 0.5 + 2.5 / RENDER_SCALE;
        }
        render_line(m_renderer, node.x, p1, node.x, p2);
    }
}

void Editor::render() {
    render_grid();
    m_banner.load_text_and_render(1, "EDITOR VIEW, controls below (F1)\n\n\n");
    if (show_help_banner) {
        update_help();
        m_banner.load_text_and_render(2, help);
    }
}

Vector2 Editor::track_point(Vector2 p) {
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

void Editor::increase_div() {
    if (2 * div <= DIV_MAX) {
        div *= 2;
        update_grid();
    }
}

void Editor::decrease_div() {
    if (0.5 * div >= DIV_MIN) {
        div *= 0.5;
        update_grid();
    }
}

void Editor::update_grid() {
    n = SCENE_HEIGHT / div + 1;
    m_grid.clear();
    m_grid.resize(n, std::vector<Vector2>(n * 2));
    for (size_t i(0); i < n; ++i) {
        for (size_t j(0); j < n * 2; ++j) {
            m_grid[i][j] = Vector2(j * div, i * div);
        }
    }
}
