#include <SDL2/SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include <random>
#include "system_state.h"
#include "scene_editor.h"
#include "render.h"
#include "utils.h"

/* Gather the variables that control the simulation and
 * need to be saved outside the loop of the program.
 */
struct ControlData {
    enum class BodyType { BALL, RECTANGLE, TRIANGLE, FRAME };

    ControlData();

    bool simulation_enabled;
    bool slow_motion_enabled;
    bool quit;
    bool editor_view;
    bool adding_link;
    bool movable_body;
    BodyType body_type;
    double px_0;
    double py_0;
    Vector2 pointer;
    Vector2 new_pointer;
};

ControlData::ControlData()
:   simulation_enabled(false),
    slow_motion_enabled(false),
    quit(false),
    editor_view(false),
    adding_link(false),
    movable_body(true),
    body_type(BodyType::BALL),
    px_0(0),
    py_0(0),
    pointer(0, 0),
    new_pointer(0, 0)
{}

const SDL_Color text_color({255, 255, 255, 255});

unsigned SCREEN_WIDTH;
unsigned SCREEN_HEIGHT;
double SCENE_WIDTH;
double SCENE_HEIGHT;
extern const double RENDER_SCALE(100.0);
constexpr unsigned SCREEN_FPS(120);
constexpr unsigned SCREEN_TICKS_PER_FRAME(1000 / SCREEN_FPS);
constexpr unsigned STEPS_PER_FRAME(20);
constexpr double DIVISION(0.25);

void close(VTextLayout& layout, LTexture& texture);
void handle_events(SDL_Event& e, SystemState& lab, double dt, ControlData& control);


SDL_Window* window(nullptr);
SDL_Renderer* renderer(nullptr);
TTF_Font* font(nullptr);
TTF_Font* scale_font(nullptr);


int main(int argc, char* argv[]) {
    srand((unsigned) time(0));

    Uint64 now(SDL_GetPerformanceCounter());
    Uint64 old(0);
    double delta_time(0);
    
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Mechanical physics simulation",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    SDL_DisplayMode dm;
    int w, h;
    SDL_GetRendererOutputSize(renderer, &w, &h);
    std::cout << w << " x " << h << "\n";
    SCREEN_WIDTH = w;
    SCREEN_HEIGHT = h;
    SCENE_WIDTH = SCREEN_WIDTH / RENDER_SCALE;
    SCENE_HEIGHT = SCREEN_HEIGHT / RENDER_SCALE;
    if (SDL_GetDesktopDisplayMode(0, &dm) != 0) {
        SDL_Log("SDL_GetDesktopDisplayMode failed: %s", SDL_GetError());
        return 1;
    }
    std::cout << dm.w << " x " << dm.h << " " << dm.refresh_rate << "\n";

    TTF_Init();
    font = TTF_OpenFont("fonts/ProggyClean.ttf", 16); //HackRegular 10
    scale_font = TTF_OpenFont("fonts/ProggyClean.ttf", 14);
    if (!font || !scale_font)
        std::cerr << "Failed to load the font!\n";
    
    
    VTextLayout main_panel(renderer, 10, 10);
    main_panel.add_texture(text_color, font);
    main_panel.add_texture(text_color, font);
    // LTexture fps_texture(renderer);
    // LTexture data_texture(renderer);
    VTextLayout video_info(renderer, 10, SCREEN_HEIGHT - 20);
    video_info.add_texture(text_color, font);
    LTexture scale_texture(renderer);
    std::string scale_text(truncate_to_string(100 / (double)RENDER_SCALE, 1000));
    scale_text += " m";
    scale_texture.load_from_rendered_text(scale_text, text_color, scale_font);

    LTimer fps_timer;
    LTimer cap_timer;
    int frames_count(0);
    fps_timer.start();

    SystemState universe;
    SceneEditor editor(DIVISION);

    SDL_Event e;
    ControlData control;

    while (!control.quit) {
        cap_timer.start();
        // Delta time calculation
        old = now;
        now = SDL_GetPerformanceCounter();
        delta_time = (double)((now - old)*1000 / (double)SDL_GetPerformanceFrequency());

        SDL_SetRenderDrawColor(renderer, 16, 16, 16, 255);
        SDL_RenderClear(renderer);

        while (SDL_PollEvent(&e) != 0) {
            handle_events(e, universe, delta_time, control);
        }

        // Frame rate capping to the desired FPS
        if (frames_count >= 1000) {
            frames_count = 0;
            fps_timer.start();
        }
        float avg_fps(frames_count / (fps_timer.get_ticks() / 1000.0));
        if (avg_fps > 2e6)
            avg_fps = 0;
        // Display FPS counter
        std::stringstream time_text;
        time_text.str("");
        time_text << "Average FPS (cap " << SCREEN_FPS << ") : " << floor(avg_fps)
                  << "\nDelta time : " << delta_time << " ms"
                  << "\nFreq : " << STEPS_PER_FRAME * avg_fps << " Hz"
                  << "\nSteps : " << STEPS_PER_FRAME
                  << "\n\n" << (control.simulation_enabled ? "RUNNING" : "PAUSE");
        // fps_texture.load_from_rendered_text(time_text.str(), text_color, font);
        // fps_texture.render(10, 10);
        main_panel.load_text_and_render(1, time_text.str());
        // Display simulation data
        // data_texture.load_from_rendered_text(universe.dump_object_data(), text_color, font);
        // data_texture.render(10, 10 + fps_texture.get_height());
        main_panel.load_text_and_render(2, universe.dump_data());
        std::stringstream video_text;
        video_text.str("");
        video_text << "speed : " << (control.slow_motion_enabled ? 0.1 : 1) << "x";
        video_info.load_text_and_render(1, video_text.str());
        
        // Physics simulation process 
        if (control.simulation_enabled) {
            if (!control.slow_motion_enabled)
                universe.process(delta_time / 1000.0, STEPS_PER_FRAME);// , avg_fps >= 0.5*SCREEN_FPS);
                        //  , universe.get_body_count() < 200);
            else
                universe.process(delta_time / 10000.0, STEPS_PER_FRAME);//, avg_fps >= SCREEN_FPS / 2);
        }

        // Rendering
        universe.render(renderer);
        if (control.editor_view) {
            control.new_pointer = editor.track_point(control.pointer);
            editor.render_grid(renderer);
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

void handle_events(SDL_Event& e, SystemState& lab, double dt, ControlData& control) {
    if (e.type == SDL_QUIT)
        control.quit = true;
    else if (e.type == SDL_KEYDOWN) {
        switch (e.key.keysym.sym) {
            case SDLK_q:
                control.quit = true; break;
            case SDLK_SPACE:
                control.simulation_enabled = !control.simulation_enabled; break;
            case SDLK_1:
                control.slow_motion_enabled = !control.slow_motion_enabled; break;
            case SDLK_2:
                control.editor_view = !control.editor_view; break;
            case SDLK_s:
                if (!control.simulation_enabled)
                    lab.process(dt / 1000, STEPS_PER_FRAME);
                break;
            case SDLK_g:
                lab.toggle_gravity(); break;
            case SDLK_n:
                lab.focus_next(); break;
            case SDLK_p:
                lab.focus_prev(); break;
            case SDLK_m:
                control.movable_body = !control.movable_body; break;
            case SDLK_b: 
                control.body_type = ControlData::BodyType::BALL; break;
            case SDLK_r:
                control.body_type = ControlData::BodyType::RECTANGLE; break;
            case SDLK_t:
                control.body_type = ControlData::BodyType::TRIANGLE; break;
            case SDLK_f:
                control.body_type = ControlData::BodyType::FRAME; break;
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
            default: break;
        }
    }
    else if (e.type == SDL_MOUSEBUTTONDOWN) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        double px = x * SCENE_WIDTH / (double)SCREEN_WIDTH;
        double py = SCENE_HEIGHT - y * SCENE_HEIGHT / (double)SCREEN_HEIGHT;
        if (control.editor_view) {
            px = control.new_pointer.x;
            py = control.new_pointer.y;
        }
        if (e.button.button == SDL_BUTTON_LEFT) {
            switch (control.body_type) {
                case ControlData::BodyType::BALL:
                    if (!control.editor_view)
                        lab.add_ball({px, py}, 1, (25) / (double)RENDER_SCALE, 
                                control.movable_body);
                    else {
                        lab.add_ball({px, py}, 1, DIVISION * 0.5, control.movable_body);
                    }
                    break;
                case ControlData::BodyType::RECTANGLE:
                    if (!control.editor_view)
                        lab.add_rectangle({px, py}, 3, (10 + rand() % 100) / (double)RENDER_SCALE, 
                                                   (10 + rand() % 100) / (double)RENDER_SCALE,
                                                   control.movable_body);
                    else {
                        lab.add_rectangle({px, py}, 3, DIVISION, DIVISION, control.movable_body);
                    }
                    break;
                case ControlData::BodyType::TRIANGLE:
                    break;
                case ControlData::BodyType::FRAME:
                    lab.add_frame({px, py});
                    break;
                default:
                    break;
            }
        }else if (e.button.button == SDL_BUTTON_RIGHT) {
            if (!control.adding_link) {
                control.px_0 = px;
                control.py_0 = py;
                control.adding_link = true;
            }else {
                lab.add_link(Vector2(control.px_0, control.py_0), Vector2(px, py));
                control.adding_link = false;
            }
        }else if (e.button.button == SDL_BUTTON_MIDDLE)
            lab.focus_on_position(Vector2(px, py));
    }else if (e.type == SDL_MOUSEMOTION) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        double px = x * SCENE_WIDTH / (double)SCREEN_WIDTH;
        double py = SCENE_HEIGHT - y * SCENE_HEIGHT / (double)SCREEN_HEIGHT;
        if (control.pointer.x != px || control.pointer.y != py) {
            control.pointer = Vector2(px, py);
        }
    }
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

