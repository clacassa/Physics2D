#ifndef RENDER_H
#define RENDER_H

#include <SDL_render.h>

struct Vector2;

const SDL_Color bg_color({31, 31, 31, 255});
const SDL_Color text_color({255, 255, 255, 255});
const SDL_Color kinematic_body_color({189, 183, 107, 255});
const SDL_Color dynamic_body_color({255, 180, 180, 255});
const SDL_Color focus_color({255, 0, 255, 255});

extern const unsigned SCREEN_WIDTH;
extern const unsigned SCREEN_HEIGHT;

extern const double SCENE_WIDTH;          // Scene width in meters
extern double RENDER_SCALE;                 // Pixel to meter ratio (#of px for 1m)
extern const double SCENE_HEIGHT;

extern const unsigned SCREEN_FPS;
// extern const unsigned SCREEN_TICKS_PER_FRAME;

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
    Vector2 world_to_screen(Vector2 world_p);
    Vector2 screen_to_world(int px, int py);
    void translate_screen_x(int dx);
    void translate_screen_y(int dy);
    void translate_world(Vector2 delta);
    void zoom_in();
    void zoom_out();
}

#endif /* RENDER_H */
