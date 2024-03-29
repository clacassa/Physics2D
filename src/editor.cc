#include <imgui.h>
#include <iostream>
#include <climits>
#include "editor.h"
#include "render.h"
#include "rigid_body.h"

Editor::Editor(SDL_Renderer* renderer, TTF_Font* font, double division)
:   div(division),
    n(SCENE_HEIGHT / div + 1),
    active_node(0, 0),
    m_renderer(renderer),
    m_font(font),
    m_title(m_renderer, text_color, m_font),
    m_banner(m_renderer, 10), // centered layout
    body_shape(BALL),
    body_type(DYNAMIC),
    enabled_body(true),
    show_help_banner(true),
    spring_very_stiff(false),
    current_stiffness(spring_default_stiffness),
    damping(Spring::UNDERDAMPED)
{
    m_grid.resize(n, std::vector<Vector2>(n * 2));
    for (size_t i(0); i < n; ++i) {
        for (size_t j(0); j < n * SCENE_WIDTH / SCENE_HEIGHT; ++j) {
            m_grid[i][j] = Vector2(j * div, i * div);
        }
    }

    m_banner.add_texture(text_color, m_font);
    m_banner.add_texture(text_color, m_font);
    update_help();
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

void Editor::render_grid() {
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
    for (auto row : m_grid) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }
    // Render the cross pointer
    SDL_SetRenderDrawColor(m_renderer, 255, 0, 255, 127);
    double x(active_node.x);
    double y(active_node.y);
    render_line(m_renderer, {x - 10 / RENDER_SCALE, y}, {x + 10 / RENDER_SCALE, y});
    render_line(m_renderer, {x, y - 10 / RENDER_SCALE}, {x, y + 10 / RENDER_SCALE});
    // Render the median axis with graduations
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    const double x_axis_height(SCREEN_HEIGHT * 0.5);
    SDL_RenderDrawLineF(m_renderer, 0, SCREEN_HEIGHT * 0.5, SCREEN_WIDTH, SCREEN_HEIGHT * 0.5);
    SDL_RenderDrawLineF(m_renderer, SCREEN_WIDTH * 0.5, 0, SCREEN_WIDTH * 0.5, SCREEN_HEIGHT);
    double p1, p2;
    for (auto row : m_grid) {
        if (row[0].y - floor(row[0].y) == 0) {
            p1 = SCENE_WIDTH * 0.5 - 7.5 / RENDER_SCALE;
            p2 = SCENE_WIDTH * 0.5 + 7.5 / RENDER_SCALE;
        }else {
            p1 = SCENE_WIDTH * 0.5 - 2.5 / RENDER_SCALE;
            p2 = SCENE_WIDTH * 0.5 + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {p1, row[0].y}, {p2, row[0].y});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
        //render_line(m_renderer, 0, row[0].y, SCENE_WIDTH, row[0].y);
    }
    for (auto node : m_grid[0]) {
        if (node.x - floor(node.x) == 0) {
            p1 = x_axis_height - 7.5 / RENDER_SCALE;
            p2 = x_axis_height + 7.5 / RENDER_SCALE;
        }else {
            p1 = x_axis_height - 2.5 / RENDER_SCALE;
            p2 = x_axis_height + 2.5 / RENDER_SCALE;
        }
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {node.x, p1}, {node.x, p2});
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 63);
        //render_line(m_renderer, node.x, 0, node.x, SCENE_HEIGHT);
    }
}

void Editor::imgui_controls(bool* editor_active) {
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
        int new_body_shape(body_shape);
        ImGui::SeparatorText("Body type");
        ImGui::RadioButton("Ball (B)", &new_body_shape, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Rectangle (R)", &new_body_shape, 1);
        body_shape = static_cast<BodyShape>(new_body_shape);

        static int new_body_type(1);
        const char* items_type("Static\0Dynamic\0");
        ImGui::Combo("Type", &new_body_type, items_type);
        body_type = static_cast<BodyType>(new_body_type);

        ImGui::Checkbox("enabled", &enabled_body);

        ImGui::SeparatorText("Spring type");
        ImGui::Checkbox("Infinitely stiff", &spring_very_stiff);
        static float k(spring_very_stiff ? spring_infnite_stiffness : spring_default_stiffness);
        ImGui::BeginDisabled(spring_very_stiff);
        if (spring_very_stiff) {
            k = spring_infnite_stiffness;
        }
        ImGui::InputFloat("Stiffness", &k);
        ImGui::EndDisabled();
        current_stiffness = k;

        static int new_damping_type(0);
        // const char* items[] = {"Undamped", "Underdamped", "Critically damped", "Overdamped"};
        // const char* combo_label(items[new_damping_type]);
        // if (ImGui::BeginCombo("Damping", combo_label)) {
        //     for (int n(0); n < IM_ARRAYSIZE(items); ++n) {
        //         const bool is_selected(new_damping_type == n);
        //         if (ImGui::Selectable(items[n], is_selected))
        //             new_damping_type = n;

        //         if (is_selected)
        //             ImGui::SetItemDefaultFocus();
        //     }
        //     ImGui::EndCombo();
        // }
        const char* items_damping("Undamped\0Underdamped\0Critically damped\0Overdamped\0");
        ImGui::Combo("Damping", &new_damping_type, items_damping);
        damping = static_cast<Spring::DampingType>(new_damping_type);

        ImGui::SeparatorText("Layout");
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Larger grid ");
        ImGui::SameLine();
        if (ImGui::ArrowButton("> grid", ImGuiDir_Up)) {
            decrease_div();    
        }
        ImGui::Text("Smaller grid");
        ImGui::SameLine();
        if (ImGui::ArrowButton("< grid", ImGuiDir_Down)) {
            increase_div();
        }
    }
    ImGui::End();
}

void Editor::update_help() {
    help = "<: Smaller grid       >: Larger grid       LMB: Add a body       ";
    help += "B: Ball       ";
    if (body_shape == BALL) {
        help.replace(help.length() - 6, 1, "*");
    }
    help += "R: Rectangle       ";
    if (body_shape == RECTANGLE) {
        help.replace(help.length() - 6, 1, "*");
    }
    help += "9: Static body [  ]     ";
    if (body_type == STATIC) {
        help.replace(help.length() - 9, 4, "[v/]");
    }
    help += "0: Enabled body [  ]     ";
    if (enabled_body) {
        help.replace(help.length() - 9, 4, "[v/]");
    }
    help += "\n\nRMB: Add a spring       4: Elastic [  ]     ";
    if (!spring_very_stiff) {
        help.replace(help.length() - 9, 4, "[v/]");
    }
    help += "Damping options:  5: Undamped       ";
    if (damping == Spring::UNDAMPED) {
        help.replace(help.length() - 6, 1, "*");
    }
    help += "6: Underdamped       ";
    if (damping == Spring::UNDERDAMPED) {
        help.replace(help.length() - 6, 1, "*");
    }
    help += "7: Critically damped       ";
    if (damping == Spring::CRIT_DAMPED) {
        help.replace(help.length() - 6, 1, "*");
    }
    help += "8: Overdamped       ";
    if (damping == Spring::OVERDAMPED) {
        help.replace(help.length() - 6, 1, "*");
    }
}
