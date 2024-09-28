#include <imgui.h>
#include <iostream>
#include <climits>
#include "editor.h"
#include "render.h"
#include "rigid_body.h"

Editor::Editor(SDL_Renderer* renderer, TTF_Font* font, double division)
:   div(division),
    active_node(0, 0),
    m_renderer(renderer),
    m_font(font),
    m_title(m_renderer, text_color, m_font),
    m_banner(m_renderer, 10), // centered layout
    body_type(DYNAMIC),
    body_enabled(true),
    spring_incompressible(false),
    spring_stiffness(spring_stiffness_default),
    spring_damping(Spring::UNDERDAMPED),
    show_help_banner(true)
{
    update_grid();

    m_banner.add_texture(text_color, m_font);
    m_banner.add_texture(text_color, m_font);
}

void Editor::render() {
    render_grid();
    // imgui_controls();
    m_banner.load_text_and_render(1, "EDITOR VIEW\n");
    if (!show_help_banner) {
        m_banner.load_text_and_render(2, "Press F1 to show the controls");
    }
}

Vector2 Editor::track_point(Vector2 p) {
    Vector2 tracked_point;
    double dist(INT_MAX);

    if (p.x >= 0 && p.y >= 0) {
        for (size_t i(0); i < m_first_quad.size(); ++i) {
            for (size_t j(0); j < m_first_quad[0].size(); ++j) {
                const Vector2 node(m_first_quad[i][j]);
                const Vector2 v(node - p);
                double d(v.x * v.x + v.y * v.y);
                if (d < dist) {
                    tracked_point = node;
                    dist = d;
                }
            }
        }
    }else if (p.x < 0 && p.y >= 0) {
        for (size_t i(0); i < m_second_quad.size(); ++i) {
            for (size_t j(0); j < m_second_quad[0].size(); ++j) {
                const Vector2 node(m_second_quad[i][j]);
                const Vector2 v(node - p);
                double d(v.x * v.x + v.y * v.y);
                if (d < dist) {
                    tracked_point = node;
                    dist = d;
                }
            }
        }
    }else if (p.x < 0 && p.y < 0) {
        for (size_t i(0); i < m_third_quad.size(); ++i) {
            for (size_t j(0); j < m_third_quad[0].size(); ++j) {
                const Vector2 node(m_third_quad[i][j]);
                const Vector2 v(node - p);
                double d(v.x * v.x + v.y * v.y);
                if (d < dist) {
                    tracked_point = node;
                    dist = d;
                }
            }
        }
    }else {
        for (size_t i(0); i < m_fourth_quad.size(); ++i) {
            for (size_t j(0); j < m_fourth_quad[0].size(); ++j) {
                const Vector2 node(m_fourth_quad[i][j]);
                const Vector2 v(node - p);
                double d(v.x * v.x + v.y * v.y);
                if (d < dist) {
                    tracked_point = node;
                    dist = d;
                }
            }
        }
    }

    active_node = tracked_point;
    return tracked_point;
}

void Editor::update_grid() {
    // First quadrant
    const Vector2 tr(camera::screen_to_world(SCREEN_WIDTH, 0));
    m_first_quad.clear();
    if (tr.x >= 0 && tr.y >= 0) {
        m_first_quad.resize(abs(tr.y) / div + 1, std::vector<Vector2>(abs(tr.x) / div + 1));
        for (size_t i(0); i < m_first_quad.size(); ++i) {
            for (size_t j(0); j < m_first_quad[0].size(); ++j) {
                m_first_quad[i][j] = Vector2(j * div, i * div);
            }
        }
    }
    // Second quadrant
    const Vector2 tl(camera::screen_to_world(0, 0));
    m_second_quad.clear();
    if (tl.x < 0 && tl.y >= 0) {
        m_second_quad.resize(abs(tl.y) / div + 1, std::vector<Vector2>(abs(tl.x) / div + 1));
        for (size_t i(0); i < m_second_quad.size(); ++i) {
            for (size_t j(0); j < m_second_quad[0].size(); ++j) {
                m_second_quad[i][j] = Vector2(j * -div, i * div);
            }
        }
    }
    // Third quadrant
    const Vector2 bl(camera::screen_to_world(0, SCREEN_HEIGHT));
    m_third_quad.clear();
    if (bl.x < 0 && bl.y < 0) {
        m_third_quad.resize(abs(bl.y) / div + 1, std::vector<Vector2>(abs(bl.x) / div + 1));
        for (size_t i(0); i < m_third_quad.size(); ++i) {
            for (size_t j(0); j < m_third_quad[0].size(); ++j) {
                m_third_quad[i][j] = Vector2(j * -div, i * -div);
            }
        }
    }
    // Fourth quadrant
    const Vector2 br(camera::screen_to_world(SCREEN_WIDTH, SCREEN_HEIGHT));
    m_fourth_quad.clear();
    if (br.x >= 0 && br.y < 0) {
        m_fourth_quad.resize(abs(br.y) / div + 1, std::vector<Vector2>(abs(br.x) / div + 1));
        for (size_t i(0); i < m_fourth_quad.size(); ++i) {
            for (size_t j(0); j < m_fourth_quad[0].size(); ++j) {
                m_fourth_quad[i][j] = Vector2(j * div, i * -div);
            }
        }
    }
}

void Editor::render_grid() {
    // Render the nodes
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    for (auto row : m_first_quad) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }
    for (auto row : m_second_quad) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }
    for (auto row : m_third_quad) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }
    for (auto row : m_fourth_quad) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }
    // Highlight the active node
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    render_filled_circle(m_renderer, active_node, 3 / RENDER_SCALE);
    // Render the median axis with graduations
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    const double x_axis_pos(0);
    const double x_axis_screen_pos(camera::world_to_screen({0, x_axis_pos}).y);
    const double y_axis_pos(0);
    const double y_axis_screen_pos(camera::world_to_screen({y_axis_pos, 0}).x);
    SDL_RenderDrawLineF(m_renderer, 0, x_axis_screen_pos, SCREEN_WIDTH, x_axis_screen_pos);
    SDL_RenderDrawLineF(m_renderer, y_axis_screen_pos, 0, y_axis_screen_pos, SCREEN_HEIGHT);
    double p1, p2;
    for (auto row : m_first_quad) {
        if (row.empty()) {
            continue;
        }
        if (row[0].y - floor(row[0].y) == 0) {
            p1 = y_axis_pos - 7.5 / RENDER_SCALE;
            p2 = y_axis_pos + 7.5 / RENDER_SCALE;
        }else {
            p1 = y_axis_pos - 2.5 / RENDER_SCALE;
            p2 = y_axis_pos + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {p1, row[0].y}, {p2, row[0].y});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
    }
    for (auto node : m_first_quad[0]) {
        if (node.x - floor(node.x) == 0) {
            p1 = x_axis_pos - 7.5 / RENDER_SCALE;
            p2 = x_axis_pos + 7.5 / RENDER_SCALE;
        }else {
            p1 = x_axis_pos - 2.5 / RENDER_SCALE;
            p2 = x_axis_pos + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {node.x, p1}, {node.x, p2});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
    }
    for (auto node : m_second_quad[0]) {
        if (node.x - floor(node.x) == 0) {
            p1 = x_axis_pos - 7.5 / RENDER_SCALE;
            p2 = x_axis_pos + 7.5 / RENDER_SCALE;
        }else {
            p1 = x_axis_pos - 2.5 / RENDER_SCALE;
            p2 = x_axis_pos + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {node.x, p1}, {node.x, p2});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
    }
    for (auto row : m_third_quad) {
        if (row.empty()) {
            continue;
        }
        if (row[0].y - floor(row[0].y) == 0) {
            p1 = y_axis_pos - 7.5 / RENDER_SCALE;
            p2 = y_axis_pos + 7.5 / RENDER_SCALE;
        }else {
            p1 = y_axis_pos - 2.5 / RENDER_SCALE;
            p2 = y_axis_pos + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {p1, row[0].y}, {p2, row[0].y});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
    }
}

void Editor::show_controls(bool* editor_active) {
    const float pad(10.0);
    const ImGuiViewport* viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(viewport->WorkPos);
    ImVec2 window_pos(work_pos.x + pad, work_pos.y + 250);
    ImGui::SetNextWindowPos(window_pos, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.75f);
    ImGuiWindowFlags window_flags(ImGuiWindowFlags_AlwaysAutoResize);

    // ImGui::SetNextWindowCollapsed(!show_help_banner);
    if (!show_help_banner) {
        return;
    }
    if (ImGui::Begin("Editor controls", &show_help_banner, window_flags)) {
        static int new_body_type(DYNAMIC);
        const char* items_type("Static\0Kinematic\0Dynamic\0");
        ImGui::Combo("Type", &new_body_type, items_type);
        body_type = static_cast<BodyType>(new_body_type);

        ImGui::Checkbox("enabled", &body_enabled);

        ImGui::SeparatorText("Spring type");
        ImGui::Checkbox("Infinitely stiff", &spring_incompressible);
        static float k(spring_stiffness_default);
        ImGui::BeginDisabled(spring_incompressible);
        if (spring_incompressible) {
            k = spring_stiffness_infinite;
        }
        ImGui::InputFloat("Stiffness", &k);
        ImGui::EndDisabled();
        spring_stiffness = k;

        static int new_damping_type(0);
        const char* items_damping("Undamped\0Underdamped\0Critically damped\0Overdamped\0");
        ImGui::Combo("Damping", &new_damping_type, items_damping);
        spring_damping = static_cast<Spring::DampingType>(new_damping_type);
    }
    ImGui::End();
}