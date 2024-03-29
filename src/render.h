#ifndef RENDER_H
#define RENDER_H

#include <SDL_render.h>
#include "vector2.h"

struct Vector2;

const SDL_Color bg_color({31, 31, 31, 255});
const SDL_Color text_color({255, 255, 255, 255});
const SDL_Color dynamic_body_color({255, 180, 180, 255});

constexpr double SCENE_WIDTH(5);          // Scene width in meters

extern const unsigned SCREEN_WIDTH;
extern const unsigned SCREEN_HEIGHT;
extern double RENDER_SCALE;                 // Pixel to meter ratio (#of px for 1m)
extern const double SCENE_HEIGHT;
const unsigned STATUSBAR_HEIGHT(18);   // In pixels

void render_point(SDL_Renderer* renderer, Vector2 p);
void render_line(SDL_Renderer* renderer, Vector2 p1, Vector2 p2);
void render_filled_circle(SDL_Renderer* renderer, Vector2 center, double radius);
// Render circle algorithm, very fast
void render_fill_circle_fast(SDL_Renderer*, Vector2 center, double radius);
void render_circle(SDL_Renderer* renderer, Vector2 center, double radius);
void render_rectangle(SDL_Renderer* renderer, Vector2 center, double w, double h);

/**
 *  RENDER_SCALE = camera_width
 */ 
namespace camera {
    Vector2 transform_world_to_screen(Vector2 world_p);
    Vector2 transform_screen_to_world(int px, int py);
    void translate_left();
    void translate_down();
    void translate_right();
    void translate_up();
}

#endif /* RENDER_H */
