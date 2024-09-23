#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include "application.h"
#include "rigid_body.h"
#include "utils.h"
#include "control.h"
#include "render.h"
#include "config.h"

constexpr unsigned STEPS_PER_FRAME(50);

Application::Application(SDL_Window* window, SDL_Renderer* renderer, double w, double h,
        TTF_Font* font)
:   m_window(window),
    m_renderer(renderer),
    m_width(w),
    m_height(h),
    m_font_main(font),
    m_arrow_cursor(SDL_CreateSystemCursor(SDL_SystemCursor::SDL_SYSTEM_CURSOR_ARROW)),
    m_crosshair_cursor(SDL_CreateSystemCursor(SDL_SystemCursor::SDL_SYSTEM_CURSOR_CROSSHAIR)),
    m_exit_status(0),
    m_editor(m_renderer, m_font_main, SCENE_WIDTH / 100),
    delta_time(0)
{
    // ImGui initialisation
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForSDLRenderer(m_window, m_renderer);
    ImGui_ImplSDLRenderer2_Init(m_renderer);

    ImGuiIO& io(ImGui::GetIO());
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // Center camera
    camera::translate_screen_x(SCREEN_WIDTH * 0.5);
    camera::translate_screen_y(SCREEN_HEIGHT * 0.5);
    m_editor.update_grid();

    SDL_SetCursor(m_crosshair_cursor);
}

Application::~Application() {
    TTF_CloseFont(m_font_main);
    m_font_main = nullptr;

    SDL_FreeCursor(m_arrow_cursor);
    m_arrow_cursor = nullptr;
    SDL_FreeCursor(m_crosshair_cursor);
    m_crosshair_cursor = nullptr;

    SDL_DestroyRenderer(m_renderer);
    SDL_DestroyWindow(m_window);
    m_window = nullptr;
    m_renderer = nullptr;

    TTF_Quit();
    SDL_Quit();

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}

int Application::run() {
    if (m_exit_status) {
        return m_exit_status;
    }

    VTextLayout left_panel(m_renderer, 10, 10);
    left_panel.add_texture(text_color, m_font_main);
    left_panel.add_texture(text_color, m_font_main);

    Uint64 now(SDL_GetPerformanceCounter());
    Uint64 old(0);

    LTimer fps_timer;
    LTimer cap_timer;
    int frames_count(0);
    fps_timer.start();

    // demo_collision();

    SDL_Event e;
    while (!m_ctrl.quit) {
        cap_timer.start();
        // Delta time calculation
        old = now;
        now = SDL_GetPerformanceCounter();
        delta_time = (double)((now - old)*1000 / (double)SDL_GetPerformanceFrequency());

        while (SDL_PollEvent(&e) != 0) {
            handle_event(e, delta_time);
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        // Frame rate capping to the desired FPS
        if (frames_count >= 1000) {
            frames_count = 0;
            fps_timer.start();
        }
        float avg_fps(frames_count / (fps_timer.get_ticks() / 1000.0));
        if (avg_fps > 2e6) {
            avg_fps = 0;
        }

        // Simulation
        if (m_ctrl.simulation.running) {
            double dt(delta_time / 1000.0);
            if (m_settings.slow_motion) {
                dt /= 10.0;
            }
            m_world.process(dt, STEPS_PER_FRAME, m_settings, false);
        }

        // Rendering
        ImGuiIO& io(ImGui::GetIO());
        SDL_RenderSetScale(m_renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(m_renderer, bg_color.r, bg_color.g, bg_color.b, bg_color.a);
        SDL_RenderClear(m_renderer);

        m_world.render(m_renderer, m_ctrl.simulation.running, m_settings);
        if (m_ctrl.editor.active) {
            m_editor.render();
            m_editor.show_controls(&m_ctrl.editor.active);
        }

        if (m_ctrl.editor.adding_spring) {
            SDL_SetRenderDrawColor(m_renderer, 0, 128, 255, 255);
            render_line(m_renderer, m_ctrl.input.prev_click, m_editor.get_active_node());
        }

        // ImGui::ShowDemoWindow();
        // show_menubar();
        show_main_overlay(avg_fps);
        show_settings_panel();
        bool property_open(true);
        show_property_editor(&property_open);
        ImGui::Render();

        // Render additional platform windows
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            std::cout << "lol\n";
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
        }

     	ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
        // Refresh renderer
        SDL_RenderPresent(m_renderer);

        ++frames_count;
        // If frame finished early
        unsigned frame_ticks(cap_timer.get_ticks());
        if (frame_ticks < SCREEN_TICKS_PER_FRAME) {
            SDL_Delay(SCREEN_TICKS_PER_FRAME - frame_ticks);
        }
    }
    
    return m_exit_status;
}

void Application::handle_event(SDL_Event& e, const double dt) {
    ImGui_ImplSDL2_ProcessEvent(&e);
    const ImGuiIO io(ImGui::GetIO());

    if (e.type == SDL_QUIT) {
        m_ctrl.quit = true;
        return;
    }

    const double DIV(m_editor.get_div());
    if (e.type == SDL_KEYDOWN && !io.WantCaptureKeyboard) {
        switch (e.key.keysym.sym) {
            case SDLK_q:
                m_ctrl.quit = true;
                break;
            case SDLK_SPACE:
                m_ctrl.simulation.running = !m_ctrl.simulation.running;
                if (m_ctrl.editor.active) {
                    m_ctrl.editor.active = false;
                }
                break;
            case SDLK_0:
                m_world.destroy_all();
                // demo_collision();
                demo_stacking();
                break;
            case SDLK_1:
                m_ctrl.editor.active = false;
                break;
            case SDLK_2:
                m_ctrl.editor.active = true;
                m_ctrl.simulation.running = false;
                m_editor.update_grid();
                SDL_SetCursor(m_crosshair_cursor);
                break;
            case SDLK_EQUALS:
                m_settings.slow_motion = !m_settings.slow_motion;
                break;
            case SDLK_s:
                if (!m_ctrl.simulation.running)
                    m_world.process(dt / 1000, STEPS_PER_FRAME, m_settings);
                break;
            case SDLK_g:
                m_world.toggle_gravity(); break;
            case SDLK_n:
                m_world.focus_next(); break;
            case SDLK_p:
                m_world.focus_prev(); break;
            case SDLK_b: 
                m_editor.set_body_shape(Editor::BALL); break;
            case SDLK_r:
                m_editor.set_body_shape(Editor::RECTANGLE); break;
            case SDLK_RETURN:
                m_world.add_rectangle({SCENE_WIDTH / 2.0, 0.1}, 1, 0.5); break;
            case SDLK_UP:
                m_world.move_focused_body({0, DIV / 5}); break;
            case SDLK_DOWN:
                m_world.move_focused_body({0, -DIV / 5}); break;
            case SDLK_LEFT:
                m_world.move_focused_body({-DIV / 5, 0}); break;
            case SDLK_RIGHT:
                m_world.move_focused_body({DIV / 5, 0}); break;
            case SDLK_x:
                m_world.rotate_focused_body(deg2rad(-5)); break;
            case SDLK_z:
                m_world.rotate_focused_body(deg2rad(5)); break;
            case SDLK_h:
                camera::translate_screen_x(-50);
                break;
            case SDLK_j:
                camera::translate_screen_y(-50);
                break;
            case SDLK_k:
                camera::translate_screen_y(50);
                break;
            case SDLK_l:
                camera::translate_screen_x(50);
                break;
            case SDLK_LESS:
                if (m_ctrl.editor.active) {
                    if(e.key.keysym.mod == KMOD_LSHIFT) {

                    }else {

                    }
                }
                break;
            case SDLK_F1:
                if (m_ctrl.editor.active) {
                    m_editor.toggle_help();
                }
                break;
            case SDLK_DELETE:
                m_world.destroy_body();
                break;
        }
    }

    if (io.WantCaptureMouse) {
        if (SDL_GetCursor() == m_crosshair_cursor) {
            SDL_SetCursor(m_arrow_cursor);
        }
        return;
    }

    if (SDL_GetCursor() != m_crosshair_cursor) {
        SDL_SetCursor(m_crosshair_cursor);
    }

    if (e.type == SDL_MOUSEBUTTONDOWN) {
        Vector2 mouse(m_ctrl.input.pointer);
        if (m_ctrl.editor.active) {
            mouse = m_editor.get_active_node();
        }
        if (e.button.button == SDL_BUTTON_LEFT) {
            switch (m_editor.get_body_shape()) {
                case Editor::BALL:
                    if (!m_ctrl.editor.active) {
                        m_world.add_ball(mouse, SCENE_WIDTH * 0.01);
                    }else {
                        m_world.add_ball(mouse, DIV, m_editor.get_body_type(),
                                m_editor.get_enabled());
                    }
                    break;
                case Editor::RECTANGLE:
                    if (!m_ctrl.editor.active) {
                        m_world.add_rectangle(mouse, SCENE_WIDTH * (0.025 + 0.001 * (rand() % 10)), 
                                SCENE_WIDTH * (0.025 + 0.001 * (rand() % 10)));
                    }else {
                        m_world.add_rectangle(mouse, DIV * 4, DIV * 2, m_editor.get_body_type(),
                                m_editor.get_enabled());
                    }
                    break;
            }
            m_ctrl.input.prev_click = mouse;
        }else if (e.button.button == SDL_BUTTON_RIGHT) {
            if (!m_ctrl.editor.adding_spring) {
                m_ctrl.editor.adding_spring = true;
            }else {
                m_world.add_spring(m_ctrl.input.prev_click, mouse, m_editor.get_damping(),
                        m_editor.get_stiffness());
                m_ctrl.editor.adding_spring = false;
            }
            m_ctrl.input.prev_click = mouse;
        }else if (e.button.button == SDL_BUTTON_MIDDLE) {
            m_world.focus_on_position(mouse);
        }
    }else if (e.type == SDL_MOUSEMOTION) {
        int x, y;
        const Uint32 mouse_state(SDL_GetMouseState(&x, &y));
        Vector2 world_p(camera::screen_to_world(x, y));
        if (m_ctrl.input.pointer != world_p) {
            m_ctrl.input.pointer = world_p;
            m_editor.track_point(m_ctrl.input.pointer);
        }
        if (mouse_state & SDL_BUTTON_RMASK) {
            m_ctrl.editor.adding_spring = false;
            camera::translate_world(m_ctrl.input.prev_click - m_ctrl.input.pointer);
            if (m_ctrl.editor.active) {
                m_editor.update_grid();
            }
        }
    }else if (e.type == SDL_MOUSEWHEEL) {
        if (e.wheel.y > 0) {
            camera::zoom_in();
        }else {
            if (RENDER_SCALE > 0.1) {
                camera::zoom_out();
            }
        }

        if (m_ctrl.editor.active) {
            m_editor.update_grid();
        }
        
        int x, y;
        SDL_GetMouseState(&x, &y);
        Vector2 offset(camera::world_to_screen(m_ctrl.input.pointer) - Vector2(x, y));
        camera::translate_screen_x(offset.x);
        camera::translate_screen_y(-offset.y);
    }
}

void Application::demo_collision() {
    m_world.add_rectangle({-0.5, SCENE_HEIGHT * 0.5}, 0.25, 0.25, DYNAMIC, true, {3, 0});
    for (unsigned i(0); i < 250; ++i) {
        m_world.add_ball({SCENE_WIDTH * 0.25 + 0.001 * (rand() % 250),
                        SCENE_HEIGHT * 0.5 - 0.0625 + 0.001 * (rand() % 125)}, 0.01);
    }
}

void Application::demo_stacking() {
    m_world.add_rectangle({SCENE_WIDTH * 0.5, 0.5}, 5, 0.5, STATIC);
    for (int i(0); i < 5; ++i) {
        for (unsigned j(0); j < 10; ++j) {
            m_world.add_rectangle({SCENE_WIDTH * 0.5 - 0.5 * (i % 2 == 0 ? i : -i - 1), 1.5 + j}, 0.5, 0.5);
        }
    }
}

void Application::show_menubar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Menu")) {
            ImGui::MenuItem("Quit", "Q", &m_ctrl.quit);
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
}

void Application::show_main_overlay(const float avg_fps) {
    ImGuiWindowFlags flags(ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoInputs);

    // Top left corner, application metrics
    const float pad(10.0);
    const ImGuiViewport* viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(viewport->WorkPos);
    ImVec2 window_pos(work_pos.x + pad, work_pos.y + pad);
    ImGui::SetNextWindowPos(window_pos);
    ImGui::SetNextWindowBgAlpha(0.5);
    flags |= ImGuiWindowFlags_NoMove;
    bool p_open(true);
    ImGui::Begin("Main overlay", &p_open, flags);
    ImGui::Text("Average FPS (cap %.1u) : %.1f", SCREEN_FPS, floor(avg_fps));
    ImGui::Text("Delta time : %.1f ms", delta_time);
    ImGui::Text("Freq : %.1f Hz", STEPS_PER_FRAME * avg_fps);
    ImGui::Text("Steps : %.1u", STEPS_PER_FRAME);
    ImVec4 color((!m_ctrl.simulation.running), (m_ctrl.simulation.running), 0.3, 1);
    ImGui::TextColored(color, (m_ctrl.simulation.running ? "RUNNING" : "PAUSED"));
    const float offset(ImGui::GetWindowHeight() + pad);
    ImGui::End();

    // Simulation data
    bool p_show(!m_ctrl.editor.active);
    window_pos.y += offset;
    if (p_show) {
        ImGui::SetNextWindowPos(window_pos);
        ImGui::SetNextWindowBgAlpha(0.5);
        if (ImGui::Begin("Simu overlay", &p_show, flags)) {
            ImGui::SeparatorText("Simulation perf. metrics");
            ImGui::Text("%s", m_world.dump_metrics().c_str());
            ImGui::SeparatorText("General infos");
            ImGui::Text("Bodies : %.1u", m_world.get_body_count());
            ImGui::Text("System energy : %.1f J", m_world.total_energy());
            ImGui::End();
        }
    }
}

void Application::show_property_editor(bool* p_open) {
    if (!ImGui::Begin("Property Editor", p_open)) {
        ImGui::End();
        return;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(2, 2));
    static ImGuiTableFlags flags(ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody);
    if (ImGui::BeginTable("###split", 2, flags)) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        show_placeholder_object();
        ImGui::EndTable();
    }
    ImGui::PopStyleVar();
    ImGui::End();
}

void Application::show_placeholder_object() {
    RigidBody* obj(m_world.get_focused_body());
    if (!obj) {
        return;
    }
    ImGui::PushID(obj);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    bool node_open(ImGui::TreeNode("Name", "%p", obj));
    ImGui::TableSetColumnIndex(1);
    ImGui::Text(obj->has_vertices() ?  "Rectangle" : "Ball");

    if (node_open) {
        const unsigned n_rows(10);
        const char* fields[n_rows] = {
            "Mass      [kg]", 
            "Energy     [J]", 
            "PosX       [m]", 
            "PosY       [m]", 
            "VelX     [m/s]", 
            "VelY     [m/s]", 
            "Theta    [deg]", 
            "Omega  [rad/s]", 
            "Type", 
            "IsEnabled"
        };
        float values[8] = {
            (float)obj->get_mass(),
            (float)obj->energy(true),
            (float)obj->get_p().x,
            (float)obj->get_p().y,
            (float)obj->get_v().x,
            (float)obj->get_v().y,
            (float)rad2deg(obj->get_theta()),
            (float)obj->get_omega()
        };

        bool property_changed(false);
        for (unsigned i(0); i < n_rows; ++i) {
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGuiTreeNodeFlags flags(ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen 
                    | ImGuiTreeNodeFlags_Bullet);
            flags |= ImGuiTreeNodeFlags_DefaultOpen;
            ImGui::TreeNodeEx("Field", flags, "%s", fields[i]);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);

            switch (i) {
                case 0:
                case 1:
                    ImGui::Text("%.3f", values[i]);
                    break;
                case 2:
                case 3:
                case 6:
                    if (ImGui::InputFloat(fields[i], &values[i])) {
                        property_changed = true;
                    }
                    break;
                case 4:
                case 5:
                case 7:
                    if (!obj->is_static()) {
                        if (ImGui::InputFloat(fields[i], &values[i])) {
                            property_changed = true;
                        }
                    }else {
                        ImGui::Text("%.3f", values[i]);
                    }
                    break;
                case 8:
                    switch (obj->get_type()) {
                        case STATIC:
                            ImGui::Text("static");
                            break;
                        case KINEMATIC:
                            ImGui::Text("kinematic");
                            break;
                        case DYNAMIC:
                            ImGui::Text("dynamic");
                            break;
                    }
                    break;
                case 9:
                    ImGui::Text(obj->is_enabled() ? "true" : "false");
                    break;
            }

            ImGui::NextColumn();
            ImGui::PopID();
        }

        if (property_changed) {
            obj->move(Vector2(values[2], values[3]) - obj->get_p());
            obj->rotate(deg2rad(values[6]) - obj->get_theta());
            obj->linear_impulse(Vector2(values[4], values[5]) - obj->get_v());
            obj->angular_impulse(values[7] - obj->get_omega());
        }

        ImGui::TreePop();
    }
    ImGui::PopID();
}

void Application::show_settings_panel() {
    ImGui::Begin("Settings");
    ImGui::BeginGroup();
    ImGui::Checkbox("Slow motion (x0.1)", &m_settings.slow_motion);
    ImGui::Checkbox("Track body motion", &m_settings.draw_body_trajectory);
    ImGui::Checkbox("Highlight collisions", &m_settings.highlight_collisions);
    ImGui::Checkbox("Draw contact points", &m_settings.draw_contact_points);
    ImGui::Checkbox("Draw collision normal", &m_settings.draw_collision_normal);
    ImGui::Checkbox("Draw bounding boxes", &m_settings.draw_bounding_boxes);
    ImGui::Checkbox("Draw distance proxys", &m_settings.draw_distance_proxys);
    ImGui::EndGroup();
    ImGui::End();
}

void Application::show_help_panel() {
    ImGui::Begin("Help");
    ImGui::BeginGroup();
    ImGui::SeparatorText("General");
    ImGui::Text("1: Scene view (default)");
    ImGui::Text("2: Editor view");
    ImGui::EndGroup();
    ImGui::End();
}
