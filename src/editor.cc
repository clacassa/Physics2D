#include <imgui.h>
#include <iostream>
#include <climits>
#include "editor.h"
#include "render.h"
#include "rigid_body.h"
#include "control.h"

namespace {
    enum GridBaseDivision { POWER_OF_TWO = 2, METRIC = 5 };
    const GridBaseDivision grid_base_division(METRIC);
}

Editor::Editor(SDL_Renderer* renderer, double division)
:   div(division),
    active_node(0, 0),
    m_renderer(renderer),
    show_help_banner(true)
{
    update_grid();
}

void Editor::render(Control& control) {
    render_grid();

    // Highlight the active node
    if (control.editor.creating_shape || control.editor.adding_spring) {
        const SDL_Color color(editing_color);
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 255);
        render_circle_fill(m_renderer, active_node, 3 / RENDER_SCALE);
    }

    show_controls(&control.editor.active, control);
    if (control.editor.creating_shape) {
        render_body_creation(control);
    }
}

Vector2 Editor::track_point(Vector2 p) {
    Vector2 tracked_point;
    double dist(INT_MAX);

    for (auto row : m_grid) {
        for (auto node : row) {
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

void Editor::update_grid() {
    compute_division();

    const Vector2 tl(camera::screen_to_world(0, 0));
    const Vector2 br(camera::screen_to_world(SCREEN_WIDTH, SCREEN_HEIGHT));

    m_grid.clear();
    const double width(br.x - tl.x);
    const double height(tl.y - br.y);
    m_grid.resize((unsigned)(height / div) + 1, std::vector<Vector2>((unsigned)(width / div) + 1));
    for (int i(0); i < m_grid.size(); ++i) {
        for (int j(0); j < m_grid[0].size(); ++j) {
            m_grid[i][j] = Vector2(((int)(tl.x / div) + j) * div, ((int)(tl.y / div) - i) * div);
        }
    }
}

void Editor::compute_division() {
    switch (grid_base_division) {
        case GridBaseDivision::POWER_OF_TWO: {
            const double current_scene_width((double)SCREEN_WIDTH / RENDER_SCALE);
            const unsigned ratio(SCENE_WIDTH / current_scene_width);
            const unsigned inv_ratio(current_scene_width / SCENE_WIDTH);

            if ((ratio & (ratio - 1)) == 0 && ratio != 0) { // If ratio is power of 2
                div = (SCENE_WIDTH / editor_ticks_default) / ratio;
            }else if ((inv_ratio & (inv_ratio - 1)) == 0 && inv_ratio != 0) {
                div = (SCENE_WIDTH / editor_ticks_default) * inv_ratio;
            }
#ifdef DEBUG
            std::cout << "ratio: " << ratio << "\tDIV: " << div << "\n";
#endif
        }
            break;
        case GridBaseDivision::METRIC: {
            static bool deca(false);
            if (deca) {
                if (div * RENDER_SCALE > 25) {
                    div *= 0.5;
                    deca = !deca;
                }else if (div * RENDER_SCALE < 10) {
                    div *= 5;
                    deca = !deca;
                }
            }else {
                if (div * RENDER_SCALE > 50) {
                    div *= 0.2;
                    deca = !deca;
                }else if (div * RENDER_SCALE < 12.5) {
                    div *= 2;
                    deca = !deca;
                }
            }
        }
            break;
        default:
            break;
    }
}

void Editor::on_mouse_left_click(Control& control) {
    if (!control.editor.creating_shape) {
        return;
    }

    body_creator.points_set.push_back(active_node);
    ++body_creator.points_count;
    switch (body_creator.shape_id) {
        case BodyCreator::ShapeID::CIRCLE:
            if (body_creator.points_count > 1) {
                control.editor.body_creation_rdy = create_circle();
                control.editor.creating_shape = false;
                body_creator.points_set.clear();
                body_creator.points_count = 0;
            }
            break;
        case BodyCreator::ShapeID::RECTANGLE:
            if (body_creator.points_count > 1) {
                control.editor.body_creation_rdy = create_rectangle();
                control.editor.creating_shape = false;
                body_creator.points_set.clear();
                body_creator.points_count = 0;
            }
            break;
        case BodyCreator::ShapeID::POLYGON:
            body_creator.points_set.push_back(active_node);
            break;
        default:
            break;
    }
}

bool Editor::create_circle() {
    const Vector2 p0(body_creator.points_set[0]);
    const Vector2 A(body_creator.points_set[1]);
    const double radius((A - p0).norm());
    if (radius == 0) {
        return false;
    }
    body_creator.body_shape = new Circle(radius);
    body_creator.body_def.position = p0;
    return true;
}

bool Editor::create_rectangle() {
    const Vector2 p1(body_creator.points_set[0]);
    const Vector2 p2(body_creator.points_set[1]);
    const double hw(abs((p2 - p1).x) * 0.5);
    const double hh(abs((p2 - p1).y) * 0.5);
    if (hw == 0 || hh == 0) {
        return false;
    }
    body_creator.body_shape = new Polygon(create_box(hw, hh));
    body_creator.body_def.position = Vector2(std::min(p1.x, p2.x), std::min(p1.y, p2.y)) + Vector2(hw, hh);
    return true;
}

void Editor::render_grid() {
    // Render the nodes
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    for (auto row : m_grid) {
        for (auto node : row) {
            render_point(m_renderer, node);
        }
    }    
    
    // Render the median axis with ticks
    SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
    const double x_axis_pos(0);
    const double x_axis_screen_pos(camera::world_to_screen({0, x_axis_pos}).y);
    const double y_axis_pos(0);
    const double y_axis_screen_pos(camera::world_to_screen({y_axis_pos, 0}).x);
    SDL_RenderDrawLineF(m_renderer, 0, x_axis_screen_pos, SCREEN_WIDTH, x_axis_screen_pos);
    SDL_RenderDrawLineF(m_renderer, y_axis_screen_pos, 0, y_axis_screen_pos, SCREEN_HEIGHT);

    double p1, p2;
    const Vector2 tl(camera::screen_to_world(0, 0));
    const Vector2 br(camera::screen_to_world(SCREEN_WIDTH, SCREEN_HEIGHT));
    for (unsigned i(0); i < m_grid.size(); ++i) {
        const double y_ref(m_grid[i][0].y);
        double tick_size(2.5);
        if (abs((int)(tl.y / div) - (int)i) % 5 == 0) {
            SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 15);
            render_line(m_renderer, {tl.x, y_ref}, {br.x, y_ref});
            tick_size = 7.5;
            
        }
        p1 = y_axis_pos - tick_size / RENDER_SCALE;
        p2 = y_axis_pos + tick_size / RENDER_SCALE;
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {p1, y_ref}, {p2, y_ref});
    }

    for (unsigned j(0); j < m_grid[0].size(); ++j) {
        const double x_ref(m_grid[0][j].x);
        double tick_size(2.5);
        if (abs((int)(tl.x / div) + (int)j) % 5 == 0) {
            SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 15);
            render_line(m_renderer, {x_ref, tl.y}, {x_ref, br.y});
            tick_size = 7.5;
        }
        p1 = x_axis_pos - tick_size / RENDER_SCALE;
        p2 = x_axis_pos + tick_size / RENDER_SCALE;
        SDL_SetRenderDrawColor(m_renderer, 255, 255, 255, 127);
        render_line(m_renderer, {x_ref, p1}, {x_ref, p2});
    }
}

void Editor::show_controls(bool* editor_active, Control& control) {
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
    if (!ImGui::Begin("Editor controls", &show_help_banner, window_flags)) {
        ImGui::End();
        return;
    }

    if (ImGui::BeginTabBar("##tabs", ImGuiTabBarFlags_NoTooltip)) {
        if (ImGui::BeginTabItem("Body Creation Tool")) {
            ImGui::SeparatorText("Geometric Properties");
            static int shape_creation_id(0);
            const char* items_shape_creation("CIRCLE\0RECTANGLE\0SQUARE\0POLYGON\0");
            ImGui::Combo("Select a shape type", &shape_creation_id, items_shape_creation);
            body_creator.shape_id = (BodyCreator::ShapeID)shape_creation_id;
            
            ImGui::SeparatorText("Body Properties");
            static int new_body_type(DYNAMIC);
            const char* items_type("STATIC\0KINEMATIC\0DYNAMIC\0");
            ImGui::Combo("Type", &new_body_type, items_type);
            body_creator.body_def.type = static_cast<BodyType>(new_body_type);

            ImGui::Checkbox("Physics enabled", &body_creator.body_def.enabled);

            if (control.editor.creating_shape) {
                ImGui::BeginDisabled();
                if (ImGui::Button("Create Body")) {
                    control.editor.creating_shape = true;
                    control.editor.shape_creation_id = shape_creation_id;
                }
                ImGui::EndDisabled();
            }else {
                if (ImGui::Button("Create Body")) {
                    control.editor.creating_shape = true;
                    control.editor.shape_creation_id = shape_creation_id;
                }
                if (!control.editor.creating_shape && body_creator.points_count) {
                    body_creator.points_set.clear();
                    body_creator.points_count = 0;
                }
            }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Spring Creation Tool")) {
            ImGui::SeparatorText("Stiffness");
            ImGui::Checkbox("Infinite Stiffness", &spring_creator.incompressible);
            static float k(spring_stiffness_default);
            ImGui::BeginDisabled(spring_creator.incompressible);
            if (spring_creator.incompressible) {
                k = spring_stiffness_infinite;
            }
            ImGui::InputFloat("Stiffness", &k);
            ImGui::EndDisabled();
            spring_creator.stiffness = k;

            static int new_damping_type(0);
            const char* items_damping("Undamped\0Underdamped\0Critically damped\0Overdamped\0");
            ImGui::Combo("Damping", &new_damping_type, items_damping);
            spring_creator.damping_type = static_cast<Spring::DampingType>(new_damping_type);

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void Editor::render_body_creation(Control& control) {
    switch (body_creator.shape_id) {
        case BodyCreator::ShapeID::CIRCLE: 
        if (body_creator.points_count > 0) {
            const Vector2 p1(body_creator.points_set[0]);
            const Vector2 p2(active_node);
            const double radius((p2 - p1).norm());
            SDL_SetRenderDrawColor(m_renderer, editing_color.r, editing_color.g, editing_color.b, 255);
            render_circle(m_renderer, p1, radius);
            render_line(m_renderer, p1, p2);
        }
            break;
        case BodyCreator::ShapeID::RECTANGLE:
            if (body_creator.points_count > 0) {
                const Vector2 p1(body_creator.points_set[0]);
                const Vector2 p2(active_node);
                const float w((p2 - p1).x);
                const float h((p2 - p1).y);
                const Vector2 center(p1 + Vector2(w * 0.5, h * 0.5));
                SDL_SetRenderDrawColor(m_renderer, editing_color.r, editing_color.g, editing_color.b, editing_color.a);
                render_rectangle(m_renderer, center, w, h);
                render_line(m_renderer, p1, p2);
            }
            break;
        case BodyCreator::ShapeID::POLYGON:
            break;
        default:
            break;
    }
}