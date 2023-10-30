#include <SDL2/SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include "system_state.h"
#include "ODE_solver.h"
#include "render.h"
#include "utils.h"

SDL_Window* window(nullptr);
SDL_Renderer* renderer(nullptr);
TTF_Font* font(nullptr);

const SDL_Color text_color({255, 255, 255, 255});

const unsigned SCREEN_FPS(60);
const unsigned SCREEN_TICKS_PER_FRAME(1000 / SCREEN_FPS);
const unsigned STEPS_PER_FRAME(10000);

void close(LTexture& texture, LTexture& texture2);

int main(int argc, char* argv[]) {
    Uint64 now(SDL_GetPerformanceCounter());
    Uint64 old(0);
    double delta_time(0);
    
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Mechanical physics simulation",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            SDL_WINDOW_SHOWN);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    TTF_Init();
    font = TTF_OpenFont("fonts/ProggyClean.ttf", 16); //HackRegular 10
    if (!font)
        std::cerr << "Failed to load the font!\n";
    
    LTexture fps_texture(renderer);
    std::stringstream time_text;
    LTexture data_texture(renderer);

    LTimer fps_timer;
    LTimer cap_timer;
    int frames_count(0);
    fps_timer.start();

    SystemState universe;
    EulerSolver solver;
    universe.initialize(&solver);

    SDL_Event e;

    bool quit(false);
    bool simu_enabled(false);
    bool adding_link(false);
    double px_0(0.0);
    double py_0(0.0);
    bool body_mode(false);
    while (!quit) {
        cap_timer.start();
        // Delta time calculation
        old = now;
        now = SDL_GetPerformanceCounter();
        delta_time = (double)((now - old)*1000 / (double)SDL_GetPerformanceFrequency());

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT)
                quit = true;
            else if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_q)
                    quit = true;
                else if (e.key.keysym.sym == SDLK_SPACE)
                    simu_enabled = !simu_enabled;
                else if (e.key.keysym.sym == SDLK_s && !simu_enabled)
                    universe.process(delta_time / 1000, STEPS_PER_FRAME);
                else if (e.key.keysym.sym == SDLK_g)
                    universe.toggle_gravity();
                else if (e.key.keysym.sym == SDLK_n)
                    universe.focus_next();
                else if (e.key.keysym.sym == SDLK_p)
                    universe.focus_prev();
                else if (e.key.keysym.sym == SDLK_b)
                    body_mode = true;
            }
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                double px = x * SCENE_WIDTH / (double)SCREEN_WIDTH;
                double py = SCENE_HEIGHT - y * SCENE_HEIGHT / (double)SCREEN_HEIGHT;
                if (e.button.button == SDL_BUTTON_LEFT) {
                    if (body_mode)
                        universe.add_body(Vector2{px, py});
                    else
                        universe.add_frame(Vector2{px, py});
                    adding_link = false;
                }else if (e.button.button == SDL_BUTTON_RIGHT) {
                    if (!adding_link) {
                        px_0 = px;
                        py_0 = py;
                        adding_link = true;
                    }else {
                        universe.add_link(Vector2{px_0, py_0}, Vector2{px, py});
                        adding_link = false;
                    }
                }
            }
        }

        // Frame rate capping to 60 FPS
        float avg_fps(frames_count / (fps_timer.get_ticks() / 1000.0));
        if (avg_fps > 2e6)
            avg_fps = 0;
        // Display FPS counter
        time_text.str("");
        time_text << "Average FPS (cap " << SCREEN_FPS << ") : " << avg_fps 
                  << "\nDelta time : " << delta_time << " ms"
                  << "\nFreq : " << STEPS_PER_FRAME * avg_fps << " Hz";
        fps_texture.load_from_rendered_text(time_text.str(), text_color, font);
        fps_texture.render(10, 10);
        // Display simulation data
        data_texture.load_from_rendered_text(universe.dump_object_data(), text_color, font);
        data_texture.render(10, 10 + fps_texture.get_height());

        // Physics simulation process 
        if (simu_enabled)
            universe.process(delta_time / 1000.0, STEPS_PER_FRAME);

        universe.render(renderer);
        SDL_RenderPresent(renderer);

        ++frames_count;
        // If frame finished early
        unsigned frame_ticks(cap_timer.get_ticks());
        if (frame_ticks < SCREEN_TICKS_PER_FRAME)
            SDL_Delay(SCREEN_TICKS_PER_FRAME - frame_ticks);
    }
 
    close(fps_texture, data_texture);
    return 0;
}

void close(LTexture& texture, LTexture& texture2) {
    texture.free();
    texture2.free();
    TTF_CloseFont(font);
    font = nullptr;

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    window = nullptr;
    renderer = nullptr;

    TTF_Quit();
    IMG_Quit();
    SDL_Quit();
}

