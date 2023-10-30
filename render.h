#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>

constexpr unsigned SCREEN_WIDTH(1000);
constexpr unsigned SCREEN_HEIGHT(500);

constexpr unsigned RENDER_SCALE(100);
// Aspect ratio must be 2
constexpr unsigned SCENE_WIDTH(SCREEN_WIDTH / RENDER_SCALE);
constexpr unsigned SCENE_HEIGHT(SCREEN_HEIGHT / RENDER_SCALE);

void render_body_circle(SDL_Renderer* renderer, double x, double y, double radius);
void render_line(SDL_Renderer* renderer, double x1, double y1, double x2, double y2);
void render_circle(SDL_Renderer*, double x, double y, double radius);

#endif /* RENDER_H */
