#include <SDL_keycode.h>
#include <cmath>
#include <cstddef>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <implot.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include "application.h"
#include "editor.h"
#include "rigid_body.h"
#include "shape.h"
#include "link.h"
#include "utils.h"
#include "control.h"
#include "render.h"
#include "vector2.h"
#include "config.h"

constexpr unsigned sim_substeps(20);

Application::Application(SDL_Window* window, SDL_Renderer* renderer, double w, double h)
:   m_window(window),
    m_renderer(renderer),
    m_width(w),
    m_height(h),
    m_arrow_cursor(SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_ARROW)),
    m_crosshair_cursor(SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_CROSSHAIR)),
    m_exit_status(0),
    m_editor(m_renderer, SCENE_WIDTH / 100),
    frame_time(0),
    time_step(1.0 / 60.0)
{
    // ImGui initialisation
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui_ImplSDL2_InitForSDLRenderer(m_window, m_renderer);
    ImGui_ImplSDLRenderer2_Init(m_renderer);

    ImGuiIO& io(ImGui::GetIO());
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // Center camera
    camera::translate_screen_x(SCREEN_WIDTH * 0.5);
    camera::translate_screen_y(SCREEN_HEIGHT * 0.5);
    m_editor.update_grid();

    m_world.disable_walls();
    RigidBodyDef def;
    def.position = {0.5 * SCENE_WIDTH, -0.25};
    def.type = STATIC;
    Polygon h_wall_box(create_box(0.5 * SCENE_WIDTH, 0.25));
    m_world.create_body(def, h_wall_box);
    def.position = {0.5 * SCENE_WIDTH, SCENE_HEIGHT + 0.25};
    m_world.create_body(def, h_wall_box);

    Polygon v_wall_box(create_box(0.25, 0.5 * SCENE_HEIGHT));
    def.position = {-0.25, SCENE_HEIGHT * 0.5};
    m_world.create_body(def, v_wall_box);
    def.position = {SCENE_WIDTH + 0.5, 0.5 * SCENE_HEIGHT};
    m_world.create_body(def, v_wall_box);

    SDL_SetCursor(m_crosshair_cursor);
}

Application::~Application() {
    SDL_FreeCursor(m_arrow_cursor);
    m_arrow_cursor = nullptr;
    SDL_FreeCursor(m_crosshair_cursor);
    m_crosshair_cursor = nullptr;

    SDL_DestroyRenderer(m_renderer);
    SDL_DestroyWindow(m_window);
    m_window = nullptr;
    m_renderer = nullptr;

    SDL_Quit();

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

int Application::run() {
    if (m_exit_status) {
        return m_exit_status;
    }

    Uint64 now(SDL_GetPerformanceCounter());
    Uint64 old(0);

    int frames_count(0);
    float avg_fps(SCREEN_FPS);
    Timer fps_timer;

    SDL_Event e;
    while (!m_ctrl.quit) {
        // Delta time calculation
        old = now;
        now = SDL_GetPerformanceCounter();
        frame_time = (double)((now - old)*1000 / (double)SDL_GetPerformanceFrequency());

        while (SDL_PollEvent(&e) != 0) {
            parse_event(e);
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        // Mobile average FPS counter
        if (frames_count >= 80) {
            avg_fps = (frames_count / fps_timer.get_seconds());
            frames_count = 0;
            fps_timer.reset();
        }

        // Simulation
        if (m_ctrl.simulation.running) {
            double dt(time_step);
            if (m_settings.slow_motion) {
                dt /= 10.0;
            }
            
            m_world.step(dt, sim_substeps, m_settings, false);
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
        // ImPlot::ShowDemoWindow();
        // show_menubar();
        show_main_overlay(avg_fps);
        show_settings_panel();
        show_property_editor();
        if (m_settings.plot_position || m_settings.plot_velocity) {
            show_obj_dynamics_plot(m_world.get_focused_body());
        }
        if (m_settings.plot_phase_plane) {
            show_osc_dynamics_plot(spring_ptr);
        }
        body_id_changed = 0;
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
    }
    
    return m_exit_status;
}

void Application::parse_event(SDL_Event& event) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    const ImGuiIO io(ImGui::GetIO());

    if (event.type == SDL_QUIT) {
        m_ctrl.quit = true;
        return;
    }

    if (event.type == SDL_KEYDOWN && !io.WantCaptureKeyboard) {
        parse_keybd_event(event);
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

    if (event.type == SDL_MOUSEBUTTONDOWN) {
        parse_mouse_button_event(event);
    }else if (event.type == SDL_MOUSEMOTION) {
        parse_mouse_motion_event(event);
    }else if (event.type == SDL_MOUSEWHEEL) {
        parse_mouse_wheel_event(event);
    }
}

void Application::parse_keybd_event(SDL_Event& keybd_event) {
    Vector2 mouse(m_ctrl.input.pointer);
    if (m_ctrl.editor.active) {
        mouse = m_editor.get_active_node();
    }

    const double DIV(m_editor.get_div());

    switch (keybd_event.key.keysym.sym) {
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
            spring_ptr = nullptr;
            demo_stacking();
            break;
        case SDLK_9:
            m_world.destroy_all();
            spring_ptr = nullptr;
            demo_collision();
            break;
        case SDLK_8:
            m_world.destroy_all();
            spring_ptr = nullptr;
            demo_double_pendulum();
            break;
        case SDLK_7:
            m_world.destroy_all();
            spring_ptr = nullptr;
            demo_springs();
            break;
        case SDLK_6:
            m_world.destroy_all();
            spring_ptr = nullptr;
            demo_simple_pendulum();
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
            if (!m_ctrl.simulation.running) {
                m_world.step(time_step, sim_substeps, m_settings);
            }
            break;
        case SDLK_g:
            m_world.toggle_gravity();
            break;
        case SDLK_n:
            m_world.focus_next();
            body_id_changed = 1;
            break;
        case SDLK_p:
            m_world.focus_prev();
            body_id_changed = 1;
            break;
        case SDLK_b: 
            if (!m_ctrl.editor.active) {
                RigidBodyDef body_def;
                body_def.position = mouse;
                Circle ball_shape(SCENE_WIDTH * 0.01);
                m_world.create_body(body_def, ball_shape);
            }else {
                RigidBodyDef body_def;
                body_def.position = mouse;
                body_def.type = m_editor.get_body_type();
                body_def.enabled = m_editor.get_enabled();
                Circle ball_shape(DIV);
                m_world.create_body(body_def, ball_shape);
            }
            break;
        case SDLK_r:
            if (!m_ctrl.editor.active) {
                RigidBodyDef body_def;
                body_def.position = mouse;
                Polygon box_shape(create_box(0.5 * SCENE_WIDTH * (0.025 + 0.001 * (rand() % 10)), 0.5 * SCENE_WIDTH * (0.025 + 0.001 * (rand() % 10))));
                m_world.create_body(body_def, box_shape);
            }else {
                RigidBodyDef body_def;
                body_def.position = mouse;
                body_def.type = m_editor.get_body_type();
                body_def.enabled = m_editor.get_enabled();
                Polygon box_shape(create_box(DIV * 2, DIV));
                m_world.create_body(body_def, box_shape);
            }
            break;
        case SDLK_UP:
            {
                RigidBody* body(m_world.get_focused_body());
                body->move({0, DIV / 5});
            }
            break;
        case SDLK_DOWN:
            {
                RigidBody* body(m_world.get_focused_body());
                body->move({0, -DIV / 5});
            }
            break;
        case SDLK_LEFT:
            {
                RigidBody* body(m_world.get_focused_body());
                body->move({-DIV / 5, 0});
            }
            break;
        case SDLK_RIGHT:
            {
                RigidBody* body(m_world.get_focused_body());
                body->move({DIV / 5, 0});
            }
            break;
        case SDLK_x:
            {
                RigidBody* body(m_world.get_focused_body());
                body->rotate(deg2rad(-5));
            }
            break;
        case SDLK_z:
            {
                RigidBody* body(m_world.get_focused_body());
                body->rotate(deg2rad(5));
            }
            break;
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
            if(keybd_event.key.keysym.mod == KMOD_LSHIFT) {
                camera::zoom_in();
            }else if (RENDER_SCALE > 0.1) {
                camera::zoom_out();
            }
            
            if (m_ctrl.editor.active) {
                m_editor.update_grid();
            }
            break;
        case SDLK_F1:
            if (m_ctrl.editor.active) {
                m_editor.toggle_help();
            }
            break;
        case SDLK_DELETE:
            {
                RigidBody* body(m_world.get_focused_body());
                m_world.destroy_body(body);
            }
            break;
        case SDLK_RETURN:
        {
            RigidBodyDef def;
            def.position = {SCENE_WIDTH * 0.5, SCENE_HEIGHT * 0.5};
            Vertices points;
            points[0] = {-1, 1};
            points[1] = {-1.1, 0};
            points[2] = {0, -0.35};
            points[3] = {2, 0.5};
            ConvexHull hull;
            hull.points = points;
            hull.count = 4;
            Polygon poly(hull);
            m_world.create_body(def, poly);
        }
        case SDLK_ESCAPE:
            m_world.focus_at(-1);
            body_id_changed = 1;
            break;
    }
}

void Application::parse_mouse_button_event(SDL_Event& mouse_event) {
    Vector2 mouse(m_ctrl.input.pointer);
    if (m_ctrl.editor.active) {
        mouse = m_editor.get_active_node();
    }
    
    switch (mouse_event.button.button) {
        case SDL_BUTTON_LEFT:
            body_id_changed = m_world.focus_on_position(mouse);
            if (!body_id_changed) {
                spring_ptr = m_world.get_spring_from_mouse(mouse);
            }
            break;
        case SDL_BUTTON_RIGHT:
            if (!m_ctrl.editor.adding_spring) {
                m_ctrl.editor.adding_spring = true;
            }else {
                m_world.add_spring(m_ctrl.input.prev_click, mouse, m_editor.get_damping(),
                        m_editor.get_stiffness());
                m_ctrl.editor.adding_spring = false;
            }
            break;
    }
    
    m_ctrl.input.prev_click = mouse;
}

void Application::parse_mouse_motion_event(SDL_Event& motion_event) {
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
}

void Application::parse_mouse_wheel_event(SDL_Event& wheel_event) {
    if (wheel_event.wheel.y > 0) {
        camera::zoom_in();
    }else {
        if (m_ctrl.editor.active) {
            if (RENDER_SCALE > 25) {
                camera::zoom_out();
            }
        }
        else if (RENDER_SCALE > 0.1) {
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

void Application::demo_collision() {
    RigidBodyDef body_def;
    body_def.position = {SCENE_WIDTH * 0.1, SCENE_HEIGHT * 0.5};
    body_def.velocity = {3, 0};
    Polygon collider(create_box(0.125, 0.125));
    m_world.create_body(body_def, collider);

    body_def.velocity = vector2_zero;
    for (unsigned i(0); i < 400; ++i) {
        body_def.position = {SCENE_WIDTH * 0.25 + 0.001 * (rand() % 500),
                            SCENE_HEIGHT * 0.5 - 0.125 + 0.001 * (rand() % 250)};
        Circle ball(0.01);
        m_world.create_body(body_def, ball);
    }

    m_world.disable_gravity();
    m_world.disable_walls();
    m_settings.draw_body_trajectory = 0;
}

void Application::demo_stacking() {
    RigidBodyDef body_def;
    body_def.position = {SCENE_WIDTH * 0.5, 0.5};
    body_def.type = STATIC;

    const double block_size(0.1);
    Polygon ground_box(create_box(block_size * 40, 0.1));
    RigidBody* ground(m_world.create_body(body_def, ground_box));

    body_def.type = DYNAMIC;
    for (int i(0); i < 9; ++i) {
        for (unsigned j(0); j < 15; ++j) {
            const double x(ground->get_p().x - 2*block_size * (i % 2 == 0 ? i : -i - 1));
            body_def.position = {x, 1.5 + j};
            Polygon square_box(create_square(block_size));
            m_world.create_body(body_def, square_box);
        }
    }
    m_world.enable_gravity();
    m_world.disable_walls();
    m_settings.draw_body_trajectory = 0;
}

void Application::demo_double_pendulum() {
    RigidBodyDef body_def;
    body_def.position = {SCENE_WIDTH * 0.5, SCENE_HEIGHT * 0.5};
    body_def.type = STATIC;
    body_def.enabled = false;
    Polygon anchor_box(create_box(0.5, 0.25));
    RigidBody* anchor(m_world.create_body(body_def, anchor_box));

    body_def.position = anchor->get_p() + Vector2(3, 0);
    body_def.type = DYNAMIC;
    body_def.enabled = true;
    Circle circle(0.2);
    RigidBody* body_1(m_world.create_body(body_def, circle));
    body_def.position = anchor->get_p() + Vector2(3, 3);
    RigidBody* body_2(m_world.create_body(body_def, circle));

    m_world.add_spring(anchor->get_p(), body_1->get_p(), Spring::UNDAMPED, spring_stiffness_infinite);
    m_world.add_spring(body_1->get_p(), body_2->get_p(), Spring::UNDAMPED, spring_stiffness_infinite);

    m_world.enable_gravity();
    m_world.disable_walls();
    m_world.focus_at(2);
    body_id_changed = 1;
    m_settings.draw_body_trajectory = 1;
}

void Application::demo_simple_pendulum() {
    RigidBodyDef body_def;
    body_def.position = {SCENE_WIDTH * 0.5, SCENE_HEIGHT * 0.5};
    body_def.type = STATIC;
    body_def.enabled = false;
    Polygon anchor_box(create_box(0.5, 0.25));
    RigidBody* anchor(m_world.create_body(body_def, anchor_box));

    const double length(3);
    body_def.position = anchor->get_p() + Vector2(0, -length);
    const double max_angle(PI);
    body_def.velocity = {(1 - cos(max_angle)) * sqrt(2*g*length), 0};
    body_def.type = DYNAMIC;
    body_def.enabled = true;
    Circle circle(0.2);
    RigidBody* body_1(m_world.create_body(body_def, circle));

    m_world.add_spring(anchor->get_p(), body_1->get_p(), Spring::UNDAMPED, spring_stiffness_infinite);

    m_world.enable_gravity();
    m_world.disable_walls();
    m_world.focus_on_position(body_1->get_p());
    m_settings.draw_body_trajectory = 1;
}

void Application::demo_springs() {
    RigidBodyDef bodydef;
    bodydef.type = STATIC;

    const double block_width(0.5);
    const double block_height(0.25);
    Polygon box(create_box(block_width, block_height));
    Circle ball(0.25);

    // Horizontal mass-spring systems
    for (unsigned i(0); i < 4; ++i) {
        Vector2 placeholder(0.6 * SCENE_WIDTH, 0.75 * SCENE_HEIGHT - block_height * i * 8);
        bodydef.type = STATIC;
        for (unsigned j(1); j < 6; ++j) {
            bodydef.position = placeholder + vector2_x * block_width * 2 * j;
            m_world.create_body(bodydef, box);
        }

        bodydef.position = placeholder + vector2_y * block_height * 2;
        RigidBody* anchor(m_world.create_body(bodydef, box));

        bodydef.position = anchor->get_p() + vector2_x * block_width * 6;
        bodydef.type = DYNAMIC;
        RigidBody* mobile_mass;
        if (i < 3) {
            mobile_mass = m_world.create_body(bodydef, ball);
        }else {
            mobile_mass = m_world.create_body(bodydef, create_square(block_height));
        }
        // Produce a natural frequency of 1 Hz
        float stiffness(4 * PI * PI * mobile_mass->get_mass());
        stiffness *= (1 + 10*(i-1));
        if (i == 0) {
            stiffness = spring_stiffness_default;
        }else if (i == 3) {
            stiffness = spring_stiffness_default * 5;
        }
        m_world.add_spring(anchor->get_p(), mobile_mass->get_p(), Spring::UNDAMPED, stiffness);
        const Vector2 x_offset(-vector2_x * 4 * block_width);
        mobile_mass->move(x_offset);
    }


    // Vertical mass-spring systems
    for (unsigned i(0); i < 4; ++i) {
        bodydef.position = {0.4 * SCENE_WIDTH - i * block_width * 4, 0.75 * SCENE_HEIGHT};
        bodydef.type = STATIC;
        RigidBody* vert_anchor(m_world.create_body(bodydef, box));

        bodydef.position = vert_anchor->get_p() - vector2_y * 0.25 * SCENE_HEIGHT;
        bodydef.type = DYNAMIC;
        Polygon hanging_box(create_square(block_height));
        RigidBody* hanging_mass(m_world.create_body(bodydef, hanging_box));

        // Produce a natural frequency of 1 Hz
        float stiffness(4 * PI * PI * hanging_mass->get_mass());
        const Spring::DampingType damping((Spring::DampingType)(3 - i));
        m_world.add_spring(vert_anchor->get_p(), hanging_mass->get_p(), damping, stiffness);
        // Pre-load the system by pulling down the spring
        hanging_mass->move(-vector2_y * 0.1 * SCENE_HEIGHT);
    }


    // Stick-and-slip
    for (int i(-20); i < 20; ++i) {
        Vector2 placeholder(0.5 * SCENE_WIDTH + 2 * block_width * i, 4 * block_height);
        bodydef.position = placeholder;
        bodydef.type = STATIC;
        m_world.create_body(bodydef, box);
    }

    bodydef.position = {0.5 * SCENE_WIDTH + 2 * 17 * block_width, 6 * block_height};
    bodydef.type = KINEMATIC;
    bodydef.velocity = {-0.5, 0};
    RigidBody* tractor(m_world.create_body(bodydef, box));

    bodydef.position = {0.5 * SCENE_WIDTH + 2 * 19 * block_width, tractor->get_p().y};
    bodydef.type = DYNAMIC;
    bodydef.velocity = vector2_zero;
    RigidBody* pulled_mass(m_world.create_body(bodydef, box));

    m_world.add_spring(tractor->get_p(), pulled_mass->get_p(), Spring::UNDAMPED, spring_stiffness_default);

    m_world.enable_gravity();
    m_world.disable_walls();
    m_settings.draw_body_trajectory = 0;
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

    // Top left corner
    const float pad(10.0);
    const ImGuiViewport* viewport(ImGui::GetMainViewport());
    ImVec2 work_pos(viewport->WorkPos);
    ImVec2 window_pos(work_pos.x + pad, work_pos.y + pad);
    ImGui::SetNextWindowPos(window_pos);
    ImGui::SetNextWindowBgAlpha(0.5);
    flags |= ImGuiWindowFlags_NoMove;
    bool p_open(true);
    ImGui::Begin("Main overlay", &p_open, flags);
    ImGui::Text("Average FPS (cap %.1u) : %.1d", SCREEN_FPS, int(avg_fps));
    ImGui::Text("Delta time : %.1f ms", frame_time);
    ImGui::Text("Freq : %.1f Hz", sim_substeps * avg_fps);
    ImGui::Text("Time step : %.4f ms", time_step);
    ImGui::Text("Steps : %.1u", sim_substeps);
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
        flags |= ImGuiWindowFlags_NoBackground;
        if (ImGui::Begin("Simu overlay", &p_show, flags)) {
            ImGui::Text("PROFILE");
            ImGui::Text("%s", m_world.dump_profile().c_str());
            ImGui::NewLine();
            ImGui::Text("GENERAL");
            ImGui::Text("Bodies : %.1u", m_world.get_body_count());
            ImGui::Text("System energy : %.1f J", m_world.total_energy());
            ImGui::End();
        }
    }
}

void Application::show_property_editor() {
    if (!ImGui::Begin("Property Editor")) {
        ImGui::End();
        return;
    }

    if (ImGui::BeginTabBar("##tabs", ImGuiTabBarFlags_NoTooltip)) {

        if (ImGui::BeginTabItem("Bodies")) {

            const int padding(2);
            if (body_id_changed) {
                ImGui::SetNextWindowScroll(ImVec2(-1.0, (ImGui::GetTextLineHeightWithSpacing() + padding * 2) * m_world.get_focus()));
            }
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(padding, padding));
            static ImGuiTableFlags flags(ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody |
            ImGuiTableFlags_ScrollY);

            if (ImGui::BeginTable("###split", 2, flags)) {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Name");
                ImGui::TableSetupColumn("Value");
                ImGui::TableHeadersRow();

                bool tree_open(false);
                for (size_t i(0); i < m_world.get_body_count(); ++i) {
                    if (show_placeholder_object(m_world.get_body_at(i))) {
                        tree_open = true;
                    }
                }
                if (!tree_open) {
                    m_world.focus_at(-1);
                }

                ImGui::EndTable();
            }

            ImGui::PopStyleVar();

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Springs")) {

            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }

    ImGui::End();
}

bool Application::show_placeholder_object(RigidBody* obj) {
    if (!obj) {
        return false;
    }
    ImGui::PushID(obj);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();

    ImGuiTreeNodeFlags flags(ImGuiTreeNodeFlags_SpanAllColumns);
    if (obj == m_world.get_focused_body()) {
        ImGui::SetNextItemOpen(true);
        flags |= ImGuiTreeNodeFlags_Selected;
        // ImGui::SetScrollY(ImGui::GetTextLineHeightWithSpacing() * id);
    }else {
        ImGui::SetNextItemOpen(false);
    }

    bool node_open(ImGui::TreeNodeEx("Name", flags, "RigidBody_%u", obj->get_id()));

    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%p", obj);

    if (node_open) {
        m_world.focus_body(obj);

        show_placeholder_shape(obj->get_shape());

        const unsigned n_rows(9);
        const char* fields[n_rows] = {
            "Mass        [kg]",
            "Inertia  [kg.m²]",
            "Energy       [J]",
            "Position     [m]",
            "Velocity   [m/s]",
            "Theta      [deg]",
            "Omega    [rad/s]",
            "Type", 
            "Enabled"
        };
        float values[9] = {
            (float)obj->get_mass(),
            (float)obj->get_I(),
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
            ImGuiTreeNodeFlags flags_child(ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen
                    | ImGuiTreeNodeFlags_Bullet);
            flags_child |= ImGuiTreeNodeFlags_DefaultOpen;
            ImGui::TreeNodeEx("Field", flags_child, "%s", fields[i]);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);

            switch (i) {
                case 0:
                case 1:
                case 2:
                    ImGui::Text("%.3f", values[i]);
                    break;
                case 3:
                    ImGui::TextColored(ImVec4(ImColor(255, 0, 0)), "X");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.55);
                    if (ImGui::InputFloat("##X", &values[i])) {
                        property_changed = true;
                    }
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(ImColor(0, 255, 0)), "Y");
                    ImGui::SameLine();
                    if (ImGui::InputFloat("##Y", &values[i+1])) {
                        property_changed = true;
                    }
                    break;
                case 4:
                    ImGui::BeginDisabled(obj->is_static());
                    ImGui::BeginGroup();

                    ImGui::TextColored(ImVec4(ImColor(255, 0, 0)), "X");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(-ImGui::GetContentRegionAvail().x * 0.55);
                    if (ImGui::InputFloat("##X", &values[i+1])) {
                        property_changed = true;
                    }
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(ImColor(0, 255, 0)), "Y");
                    ImGui::SameLine();
                    if (ImGui::InputFloat("##Y", &values[i+2])) {
                        property_changed = true;
                    }

                    ImGui::EndGroup();
                    ImGui::EndDisabled();
                    break;
                case 5:
                    if (ImGui::InputFloat(fields[i], &values[i+2])) {
                        property_changed = true;
                    }
                    break;
                case 6:
                    ImGui::BeginDisabled(obj->is_static());
                    if (ImGui::InputFloat(fields[i], &values[i+2])) {
                        property_changed = true;
                    }
                    ImGui::EndDisabled();
                    break;
                case 7:
                    {
                        int new_body_type(obj->get_type());
                        const char* items_body_type("STATIC\0KINEMATIC\0DYNAMIC\0");
                        if (ImGui::Combo("##Type", &new_body_type, items_body_type)) {
                            obj->set_type(static_cast<BodyType>(new_body_type));
                        }
                    }
                    break;
                case 8:
                    ImGui::Text(obj->is_enabled() ? "True" : "False");
                    break;
            }

            ImGui::NextColumn();
            ImGui::PopID();
        }

        if (property_changed) {
            obj->move(Vector2(values[3], values[4]) - obj->get_p());
            obj->rotate(deg2rad(values[7]) - obj->get_theta());
            obj->set_linear_vel(Vector2(values[5], values[6]));
            obj->set_angular_vel(values[8]);
        }

        ImGui::TreePop();
    }
    ImGui::PopID();
    return node_open;
}

bool Application::show_placeholder_shape(Shape* shape) {
    if (!shape) {
        return false;
    }
    ImGui::PushID(shape);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();

    ImGuiTreeNodeFlags flags(ImGuiTreeNodeFlags_SpanAllColumns);
    bool node_open(ImGui::TreeNodeEx("Name", flags, "Shape_1"));
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%p", shape);

    if (node_open) {
        const char* shape_fields[3] = { "Type", "Area", "Radius"};
        const unsigned count(shape->get_count());
        unsigned vertex_id(0);
        for (unsigned i(0); i < 3 + count; ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGuiTreeNodeFlags flags_child(ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen
            | ImGuiTreeNodeFlags_Bullet);
            flags_child |= ImGuiTreeNodeFlags_DefaultOpen;
            if (i < 3) {
                ImGui::TreeNodeEx("Field", flags_child, "%s", shape_fields[i]);
            }else {
                ImGui::TreeNodeEx("Field", flags_child, "Vertex_%u", vertex_id);
                ++vertex_id;
            }

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);

            switch (i) {
                case 0:
                    ImGui::Text(shape->get_type() == CIRCLE ? "CIRCLE" : "POLYGON");
                    break;
                case 1:
                    ImGui::Text("%.3f", shape->get_area());
                    break;
                case 2:
                    ImGui::Text("%.3f", shape->get_radius());
                    break;
            }

            if (i >= 3) {
                auto vertex(shape->get_vertices()[vertex_id]);
                ImGui::Text("{%.3f, %.3f}", vertex.x, vertex.y);
            }
        }

        ImGui::TreePop();
    }

    ImGui::PopID();
    return false;
}

void Application::show_obj_dynamics_plot(const RigidBody* obj) {
    if (!obj) {
        return;
    }

    if (!obj->is_dynamic()) {
        return;
    }

    if (!ImGui::Begin("Object Dynamics")) {
        ImGui::End();
        return;
    }

    static ScrollingBuffer pos_data_x, pos_data_y, vel_data_x, vel_data_y;
    static float t(0);
    static float history(10.0f); // Plot last 10s

    if (body_id_changed) {
        pos_data_x.erase();
        pos_data_y.erase();
        vel_data_x.erase();
        vel_data_y.erase();
    }
    if (m_ctrl.simulation.running) {
        t += ImGui::GetIO().DeltaTime;
        Vector2 pos(obj->get_p());
        Vector2 vel(obj->get_v());
        pos_data_x.add_point(t, pos.x);
        pos_data_y.add_point(t, pos.y);
        vel_data_x.add_point(t, vel.x);
        vel_data_y.add_point(t, vel.y);
    }

    ImPlotAxisFlags flags(ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
    if (m_settings.plot_position) {
        if (ImPlot::BeginPlot("##Position", ImVec2(-1,150))) {
            ImPlot::SetupAxes(nullptr, nullptr, 0, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 20);
            ImPlot::PlotLine("x(t)", &pos_data_x.data[0].x, &pos_data_x.data[0].y, pos_data_x.data.size(), 0, pos_data_x.offset, 2 * sizeof(float));
            ImPlot::PlotLine("y(t)", &pos_data_y.data[0].x, &pos_data_y.data[0].y, pos_data_y.data.size(), 0, pos_data_y.offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
    }

    if (m_settings.plot_velocity) {
        if (ImPlot::BeginPlot("##Velocity", ImVec2(-1,150))) {
            ImPlot::SetupAxes(nullptr, nullptr, 0, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 20);
            ImPlot::PlotLine("vx(t)", &vel_data_x.data[0].x, &vel_data_x.data[0].y, vel_data_x.data.size(), 0, vel_data_x.offset, 2 * sizeof(float));
            ImPlot::PlotLine("vy(t)", &vel_data_y.data[0].x, &vel_data_y.data[0].y, vel_data_y.data.size(), 0, vel_data_y.offset, 2 * sizeof(float));
            ImPlot::EndPlot();
        }
    }

    ImGui::End();
}

void Application::show_osc_dynamics_plot(const Spring* osc) {
    if (!osc) {
        return;
    }

    if (!ImGui::Begin("Oscillator Dynamics")) {
        ImGui::End();
        return;
    }

    static RollingBuffer phase_portrait;
    phase_portrait.span = 20e3;

    static const Spring* ptr(nullptr);
    if (osc != ptr) {
        phase_portrait.data.clear();
    }
    ptr = osc;

    if (m_ctrl.simulation.running) {
        const Vector2 state(osc->get_system_state());
        phase_portrait.add_point(state.x, state.y);
    }

    // const char* pi_str[] = {"-5PI", "-9/2PI", "-4PI", "-7/2PI", "-3PI", "-5/2PI", "-2PI",
    //     "-3/2PI", "-PI", "-PI/2", "0", "PI/2", "PI", "3/2PI", "2PI", "5/2PI", "3PI", "7/2PI",
    //     "4PI", "9/2PI", "5PI"
    // };
    // static double xticks[] = {-5*PI, -9/2*PI, -4*PI, -7/2*PI, -3*PI, -5/2*PI, -2*PI, -3/2*PI,
    //     -PI, -PI/2, 0, PI/2, PI, 3/2*PI, 2*PI, 5/2*PI, 3*PI, 7/2*PI, 4*PI, 9/2*PI, 5*PI
    // };
    if (ImPlot::BeginPlot("Phase Plane")) {
        ImPlotAxisFlags flags(ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
        ImPlot::TagX(osc->get_x_eq(), ImVec4(ImColor(0, 204, 102)), true);
        // ImPlot::SetupAxisTicks(ImAxis_X1, xticks, 21, pi_str);
        ImPlot::PlotLine("X", &phase_portrait.data[0].x, &phase_portrait.data[0].y, phase_portrait.data.size(), 0, 0, 2 * sizeof(float));
        ImPlot::EndPlot();
    }

    ImGui::End();
}

void Application::show_settings_panel() {
    ImGui::Begin("Settings");
    ImGui::BeginGroup();
    ImGui::Checkbox("Slow motion (x0.1)", &m_settings.slow_motion);
    ImGui::Checkbox("Track body motion", &m_settings.draw_body_trajectory);
    ImGui::Checkbox("Draw Center of mass", &m_settings.draw_center_of_mass);
    ImGui::Checkbox("Highlight collisions", &m_settings.highlight_collisions);
    ImGui::Checkbox("Draw contact points", &m_settings.draw_contact_points);
    ImGui::Checkbox("Draw collision normal", &m_settings.draw_collision_normal);
    ImGui::Checkbox("Draw bounding boxes", &m_settings.draw_bounding_boxes);
    ImGui::Checkbox("Draw distance proxys", &m_settings.draw_distance_proxys);
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Checkbox("Plot Position", &m_settings.plot_position);
    ImGui::Checkbox("Plot Velocity", &m_settings.plot_velocity);
    ImGui::Checkbox("Plot phase plane", &m_settings.plot_phase_plane);
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
