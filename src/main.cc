#include <SDL2/SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include <random>
#include "control.h"
#include "system_state.h"
#include "editor.h"
#include "render.h"
#include "utils.h"

unsigned SCREEN_WIDTH;
unsigned SCREEN_HEIGHT;
double SCENE_HEIGHT;
double RENDER_SCALE;

constexpr unsigned SCREEN_FPS(60);
constexpr unsigned SCREEN_TICKS_PER_FRAME(1000 / SCREEN_FPS);
constexpr unsigned STEPS_PER_FRAME(50);

void init_window_and_renderer();
void close(VTextLayout& layout, LTexture& texture);
void handle_events(SDL_Event& e, SystemState& lab, double dt, Control& control, Editor& editor);


SDL_Window* window(nullptr);
SDL_Renderer* renderer(nullptr);
TTF_Font* font(nullptr);
TTF_Font* scale_font(nullptr);


int main(int argc, char* argv[]) {
    srand((unsigned) time(0));

    Uint64 now(SDL_GetPerformanceCounter());
    Uint64 old(0);
    double delta_time(0);

    init_window_and_renderer();

    TTF_Init();
    font = TTF_OpenFont("fonts/ProggyClean.ttf", 16); //HackRegular 10
    // font = TTF_OpenFont("fonts/HackRegular.ttf", 14);
    // font = TTF_OpenFont("fonts/GohuFontMono-Regular.ttf", 16);
    // font = TTF_OpenFont("fonts/JetBrainsMonoRegularComplete.ttf", 14);
    scale_font = TTF_OpenFont("fonts/ProggyClean.ttf", 14);
    if (!font || !scale_font)
        std::cerr << "Failed to load the font!\n";
    
    VTextLayout help(renderer, 10);
    help.add_texture(text_color, font);
    VTextLayout main_panel(renderer, 10, 10);
    main_panel.add_texture(text_color, font);
    main_panel.add_texture(text_color, font);
    HTextLayout status_bar(renderer, 10, SCREEN_HEIGHT - 20);
    status_bar.add_texture(text_color, font);
    status_bar.add_texture(text_color, font);
    status_bar.add_texture(text_color, font);
    LTexture scale_texture(renderer);
    std::string scale_text(truncate_to_string(100 / (double)RENDER_SCALE, 1000));
    scale_text += " m";
    scale_texture.load_from_rendered_text(scale_text, text_color, font);

    LTimer fps_timer;
    LTimer cap_timer;
    int frames_count(0);
    fps_timer.start();

    SystemState laboratory;
    Editor editor(renderer, font, SCENE_WIDTH / 40);

    SDL_Event e;
    Control control;

    while (!control.quit) {
        cap_timer.start();
        // Delta time calculation
        old = now;
        now = SDL_GetPerformanceCounter();
        delta_time = (double)((now - old)*1000 / (double)SDL_GetPerformanceFrequency());

        while (SDL_PollEvent(&e) != 0) {
            handle_events(e, laboratory, delta_time, control, editor);
        }

        SDL_SetRenderDrawColor(renderer, 16, 16, 16, 255);
        SDL_RenderClear(renderer);

        // Frame rate capping to the desired FPS
        if (frames_count >= 1000) {
            frames_count = 0;
            fps_timer.start();
        }
        float avg_fps(frames_count / (fps_timer.get_ticks() / 1000.0));
        if (avg_fps > 2e6)
            avg_fps = 0;

        //// Simulation process //// 
        if (control.simulation.running) {
            if (!control.simulation.slow_motion)
                laboratory.process(delta_time / 1000.0, STEPS_PER_FRAME);// , avg_fps >= 0.5*SCREEN_FPS);
                        //, universe.get_body_count() < 500);
            else
                laboratory.process(delta_time / 10000.0, STEPS_PER_FRAME);//, avg_fps >= SCREEN_FPS / 2);
        }

        //// Rendering ////
        laboratory.render(renderer);
        if (control.editor.active) {
            editor.render();
        }
        // Display help / controls
        if (!control.editor.active) {
            std::string help_controls("1: Slow motion [  ]     ");
            if (control.simulation.slow_motion)
                help_controls.replace(help_controls.length() - 8, 2, "v/");
            help_controls += "2: Switch to editor       LMB: Add a body       ";
            help_controls += "B: Ball       ";
            if (editor.get_body_type() == Editor::BALL)
                help_controls.replace(help_controls.length() - 6, 1, "*");
            help_controls += "R: Rectangle       ";
            if (editor.get_body_type() == Editor::RECTANGLE)
                help_controls.replace(help_controls.length() - 6, 1, "*");
            help_controls += "RMB: Add a spring";
            help.load_text_and_render(1, help_controls);
        }
        // Display FPS counter
        std::stringstream time_text;
        time_text.str("");
        time_text << "Average FPS (cap " << SCREEN_FPS << ") : " << floor(avg_fps)
                  << "\nDelta time : " << delta_time << " ms"
                  << "\nFreq : " << STEPS_PER_FRAME * avg_fps << " Hz"
                  << "\nSteps : " << STEPS_PER_FRAME
                  << "\n\n" << (control.simulation.running ? "RUNNING" : "PAUSED");
        main_panel.load_text_and_render(1, time_text.str());
        // Simulation data
        if (!control.editor.active)
            main_panel.load_text_and_render(2, laboratory.dump_data());
        // StatusBar
        std::stringstream speed_text;
        speed_text.str("");
        speed_text << "speed: " << "x" << (control.simulation.slow_motion ? 0.1 : 1);
        status_bar.load_text_and_render(1, speed_text.str());
        std::stringstream pointer_info;
        pointer_info.str("");
        if (!control.editor.active) {
            pointer_info << "px: " << truncate_to_string(control.input.pointer.x, 100)
                         << ", py: " << truncate_to_string(control.input.pointer.y, 100);
        }else {
            pointer_info << "px: " << truncate_to_string(control.editor.cross_pointer.x, 100)
                         << ", py: " << truncate_to_string(control.editor.cross_pointer.y, 100);
        }
        status_bar.load_text_and_render(2, pointer_info.str(), SCREEN_WIDTH * 0.5, true);
        if (control.editor.active) {
            std::stringstream div_info;
            div_info.str("");
            div_info << editor.get_div() << " m/Div";
            status_bar.load_text_and_render(3, div_info.str(),
                   SCREEN_WIDTH - 200, true);
        }
        // Display a bar scale at the bottom left corner
        scale_texture.render(SCREEN_WIDTH - 10 - scale_texture.get_width(),
                SCREEN_HEIGHT - scale_texture.get_height());
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderDrawLine(renderer, SCREEN_WIDTH - 110, SCREEN_HEIGHT - 15,
                SCREEN_WIDTH - 10, SCREEN_HEIGHT - 15);

        // Refresh renderer
        SDL_RenderPresent(renderer);

        ++frames_count;
        // If frame finished early
        unsigned frame_ticks(cap_timer.get_ticks());
        if (frame_ticks < SCREEN_TICKS_PER_FRAME)
            SDL_Delay(SCREEN_TICKS_PER_FRAME - frame_ticks);
    }
 
    close(main_panel, scale_texture);
    return 0;
}

void handle_events(SDL_Event& e, SystemState& lab, double dt, Control& control, Editor& editor) {
    const double DIV(editor.get_div());
    if (e.type == SDL_QUIT)
        control.quit = true;
    else if (e.type == SDL_KEYDOWN) {
        switch (e.key.keysym.sym) {
            case SDLK_q:
                control.quit = true;
                break;
            case SDLK_SPACE:
                if (!control.editor.active)
                    control.simulation.running = !control.simulation.running;
                break;
            case SDLK_1:
                control.simulation.slow_motion = !control.simulation.slow_motion;
                break;
            case SDLK_2:
                control.editor.active = !control.editor.active;
                control.simulation.running = false;
                SDL_ShowCursor(!control.editor.active);
                break;
            case SDLK_4:
                if (control.editor.active)
                    editor.toggle_stiff_spring();
                break;
            case SDLK_5:
                if (control.editor.active)
                    editor.set_spring_damping(Spring::UNDAMPED);
                break;
            case SDLK_6:
                if (control.editor.active)
                    editor.set_spring_damping(Spring::UNDERDAMPED);
                break;
            case SDLK_7:
                if (control.editor.active)
                    editor.set_spring_damping(Spring::CRIT_DAMPED);
                break;
            case SDLK_8:
                if (control.editor.active)
                    editor.set_spring_damping(Spring::OVERDAMPED);
                break;
            case SDLK_9:
                if (control.editor.active)
                    editor.toggle_movable();
                break;
            case SDLK_0:
                if (control.editor.active)
                    editor.toggle_enabled();
                break;
            case SDLK_s:
                if (!control.simulation.running)
                    lab.process(dt / 1000, STEPS_PER_FRAME);
                break;
            case SDLK_g:
                lab.toggle_gravity(); break;
            case SDLK_n:
                lab.focus_next(); break;
            case SDLK_p:
                lab.focus_prev(); break;
            case SDLK_b: 
                editor.set_body_type(Editor::BALL); break;
            case SDLK_r:
                editor.set_body_type(Editor::RECTANGLE); break;
            case SDLK_RETURN:
                lab.add_rectangle({SCENE_WIDTH / 2.0, 0.1}, 1, 1, 0.5); break;
            case SDLK_UP:
                lab.move_focused_body({0, 5 / (double)RENDER_SCALE}); break;
            case SDLK_DOWN:
                lab.move_focused_body({0, -5 / (double)RENDER_SCALE}); break;
            case SDLK_LEFT:
                lab.move_focused_body({-5 / (double)RENDER_SCALE, 0}); break;
            case SDLK_RIGHT:
                lab.move_focused_body({5 / (double)RENDER_SCALE, 0}); break;
            case SDLK_x:
                lab.rotate_focused_body(-PI / 50); break;
            case SDLK_z:
                lab.rotate_focused_body(PI / 50); break;
            case SDLK_LESS:
                if (control.editor.active) {
                    if(e.key.keysym.mod == KMOD_LSHIFT)
                        editor.increase_div(); 
                    else
                        editor.decrease_div();
                }
                break;
            case SDLK_F1:
                if (control.editor.active)
                    editor.toggle_help();
                break;
            default: break;
        }
    }
    else if (e.type == SDL_MOUSEBUTTONDOWN) {
        Vector2 p(control.input.pointer);
        if (control.editor.active) {
            p = control.editor.cross_pointer;
        }
        if (e.button.button == SDL_BUTTON_LEFT) {
            switch (editor.get_body_type()) {
                case Editor::BALL:
                    if (!control.editor.active)
                        lab.add_ball(p, 1, (25) / (double)RENDER_SCALE);
                    else {
                        lab.add_ball(p, 1, DIV * 0.5, editor.get_movable(),
                                editor.get_enabled());
                    }
                    break;
                case Editor::RECTANGLE:
                    if (!control.editor.active)
                        lab.add_rectangle(p, 3, (10 + rand() % 100) / (double)RENDER_SCALE, 
                                (10 + rand() % 100) / (double)RENDER_SCALE);
                    else {
                        lab.add_rectangle(p, 3, DIV * 2, DIV, editor.get_movable(),
                                editor.get_enabled());
                    }
                    break;
            }
        }else if (e.button.button == SDL_BUTTON_RIGHT) {
            if (!control.editor.adding_spring) {
                control.input.prev_click = p;
                control.editor.adding_spring = true;
            }else {
                lab.add_spring(control.input.prev_click, p, editor.get_damping(),
                        editor.get_stiff_spring());
                control.editor.adding_spring = false;
            }
        }else if (e.button.button == SDL_BUTTON_MIDDLE)
            lab.focus_on_position(p);
    }else if (e.type == SDL_MOUSEMOTION) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        double px = x / RENDER_SCALE;
        double py = SCENE_HEIGHT - y / RENDER_SCALE;
        if (control.input.pointer.x != px || control.input.pointer.y != py) {
            control.input.pointer = Vector2(px, py);
            control.editor.cross_pointer = editor.track_point(control.input.pointer);
        }
    }
}

void init_window_and_renderer() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_DisplayMode dm;
    if (SDL_GetDesktopDisplayMode(0, &dm) != 0) {
        SDL_Log("SDL_GetDesktopDisplayMode failed: %s", SDL_GetError());
        exit(1);
    }
    std::cout << "Desktop display mode: " << dm.w << " x " << dm.h << ", "
              << dm.refresh_rate << " Hz\n";
    SCREEN_WIDTH = dm.w;
    SCREEN_HEIGHT = dm.h;

    window = SDL_CreateWindow("Mechanical physics simulation",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_FULLSCREEN_DESKTOP);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    int w, h;
    SDL_GetRendererOutputSize(renderer, &w, &h);
    std::cout << "Renderer output size: " << w << " x " << h << "\n";
    if (w != dm.w || h != dm.h) {
        std::cerr << "The renderer has unexpected dimensions\n";
        exit(1);
    }

    RENDER_SCALE = (double)SCREEN_WIDTH / SCENE_WIDTH;
    SCENE_HEIGHT = (SCREEN_HEIGHT - STATUSBAR_HEIGHT) / RENDER_SCALE;
}

void close(VTextLayout& layout, LTexture& texture) {
    layout.free_all();

    TTF_CloseFont(font);
    font = nullptr;
    TTF_CloseFont(scale_font);
    scale_font = nullptr;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    window = nullptr;
    renderer = nullptr;

    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
}

