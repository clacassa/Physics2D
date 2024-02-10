#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>

extern unsigned SCREEN_WIDTH;
extern unsigned SCREEN_HEIGHT;

extern const double RENDER_SCALE;
// Aspect ratio must be 2
extern double SCENE_WIDTH;
extern double SCENE_HEIGHT;

void render_point(SDL_Renderer* renderer, double x, double y);
void render_line(SDL_Renderer* renderer, double x1, double y1, double x2, double y2);
void render_filled_circle(SDL_Renderer* renderer, double x, double y, double radius);
// Render circle algorithm, very fast
void render_fill_circle_fast(SDL_Renderer*, double x, double y, double radius);
void render_circle(SDL_Renderer* renderer, double x, double y, double radius);
void render_rectangle(SDL_Renderer* renderer, double x, double y, double w, double h);

#endif /* RENDER_H */
