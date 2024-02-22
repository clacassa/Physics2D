#ifndef RENDER_H
#define RENDER_H

#include <SDL.h>

const SDL_Color text_color({255, 255, 255, 255});

constexpr double SCENE_WIDTH(25);           // Scene width in meters
constexpr unsigned STATUSBAR_HEIGHT(25);    // In pixels

extern unsigned SCREEN_WIDTH;
extern unsigned SCREEN_HEIGHT;
extern double RENDER_SCALE;                 // Pixel to meter ratio (#of px for 1m)
extern double SCENE_HEIGHT;

void render_point(SDL_Renderer* renderer, double x, double y);
void render_line(SDL_Renderer* renderer, double x1, double y1, double x2, double y2);
void render_filled_circle(SDL_Renderer* renderer, double x, double y, double radius);
// Render circle algorithm, very fast
void render_fill_circle_fast(SDL_Renderer*, double x, double y, double radius);
void render_circle(SDL_Renderer* renderer, double x, double y, double radius);
void render_rectangle(SDL_Renderer* renderer, double x, double y, double w, double h);

#endif /* RENDER_H */
