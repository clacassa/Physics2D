#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <iostream>
#include <random>
#include "application.h"
#include "render.h"
#include "config.h"

SDL_DisplayMode get_screen_dimensions();
void init_window_and_renderer();

const SDL_DisplayMode display_mode = get_screen_dimensions();
#ifndef DEBUG
const unsigned SCREEN_WIDTH = display_mode.w;
const unsigned SCREEN_HEIGHT = display_mode.h;
#else
const unsigned SCREEN_WIDTH = 1280;
const unsigned SCREEN_HEIGHT = 720;
#endif

const unsigned SCREEN_FPS = display_mode.refresh_rate;

const double SCENE_WIDTH = 25;
double RENDER_SCALE = (double)SCREEN_WIDTH / SCENE_WIDTH;
const double SCENE_HEIGHT = SCREEN_HEIGHT / RENDER_SCALE;

SDL_Window* window(nullptr);
SDL_Renderer* renderer(nullptr);
TTF_Font* font(nullptr);


int main(int argc, char* argv[]) {
    srand((unsigned) time(0));

    init_window_and_renderer();
    Application app(window, renderer, SCREEN_WIDTH, SCREEN_HEIGHT, font);
    return app.run();
}


void init_window_and_renderer() {
#ifdef DEBUG
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        const char* msg("Could not initialize video subsystem: %s");
        SDL_LogError(SDL_LOG_CATEGORY_VIDEO, msg, SDL_GetError());
        exit(1);
    }
#endif

    const char* title("MechaPhysics Simulation");
    const int x(SDL_WINDOWPOS_CENTERED);
    const int y(x);
#ifndef DEBUG
    const Uint32 window_flags(SDL_WINDOW_SHOWN | SDL_WINDOW_FULLSCREEN_DESKTOP);
#else
    const Uint32 window_flags(SDL_WINDOW_SHOWN);
#endif
    window = SDL_CreateWindow(title, x, y, SCREEN_WIDTH, SCREEN_HEIGHT, window_flags);
    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    int w, h;
    SDL_GetRendererOutputSize(renderer, &w, &h);
#ifndef DEBUG
    // Check that the renderer has fullscreen resolution
    if (w != display_mode.w || h != display_mode.h) {
        std::cerr << "The renderer doesn't match fullscreen resolution\n";
        // exit(1);
    }
#endif 
    std::cout << "Renderer output size: " << w << " x " << h << "\n";

    // Init and load fonts
    if (TTF_Init() != 0) {
        const char* msg("Could not initialize True Type Font runtime %s");
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, msg, SDL_GetError());
        exit(1);
    }
    font = TTF_OpenFont("fonts/ProggyClean.ttf", 16); //HackRegular 10
    // font = TTF_OpenFont("fonts/HackRegular.ttf", 14);
    // font = TTF_OpenFont("fonts/GohuFontMono-Regular.ttf", 16);
    // font = TTF_OpenFont("fonts/JetBrainsMonoRegularComplete.ttf", 14);
    if (!font) {
        std::cerr << "Failed to load the font!\n";
    }

    // SDL_Surface* icon(IMG_Load("Spacetime_curvature.jpg"));
    // SDL_SetWindowIcon(window, icon);
}

SDL_DisplayMode get_screen_dimensions() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        const char* msg("Could not initialize video subsystem: %s");
        SDL_LogError(SDL_LOG_CATEGORY_VIDEO, msg, SDL_GetError());
        exit(1);
    }

    SDL_DisplayMode dm;
    if (SDL_GetDesktopDisplayMode(0, &dm) != 0) {
        SDL_Log("SDL_GetDesktopDisplayMode failed: %s", SDL_GetError());
        exit(1);
    }

    return dm;
}

