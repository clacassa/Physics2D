#ifndef RENDER_H
#define RENDER_H

#include "vector2.h"
#include <SDL_render.h>

struct Vector2;

const SDL_Color bg_color({31, 31, 31, 255});
const SDL_Color text_color({255, 255, 255, 255});
const SDL_Color kinematic_body_color({189, 183, 107, 255});
const SDL_Color dynamic_body_color({255, 180, 180, 255});
const SDL_Color focus_color({255, 0, 255, 255});
const SDL_Color spring_color({160, 160, 160, 255});

extern const unsigned SCREEN_WIDTH;
extern const unsigned SCREEN_HEIGHT;

extern const double SCENE_WIDTH;        // Scene width in meters
extern double RENDER_SCALE;             // Pixel to meter ratio (#of px for 1m)
extern const double SCENE_HEIGHT;

extern const unsigned SCREEN_FPS;

void render_point(SDL_Renderer* renderer, Vector2 p);
void render_line(SDL_Renderer* renderer, Vector2 p1, Vector2 p2);
void render_circle(SDL_Renderer* renderer, Vector2 center, double radius);
void render_circle_fill(SDL_Renderer*, Vector2 center, double radius);
void render_circle_fill_raster(SDL_Renderer* renderer, Vector2 center, double radius);
void render_rectangle(SDL_Renderer* renderer, Vector2 center, float w, float h);
void render_polygon_fill(SDL_Renderer* renderer, Vector2* vertices, uint8_t n, uint32_t color);

/**
 *  RENDER_SCALE = camera_width
 */ 
namespace camera {
    // Screen<->World conversions
    Vector2 world_to_screen(Vector2 world_p);
    Vector2 screen_to_world(int px, int py);

    // Camera position control
    void translate_screen_x(int dx);
    void translate_screen_y(int dy);
    void translate_world(Vector2 delta);
    void set_position(const Vector2 pos = vector2_zero);

    // Camera scale control
    void zoom_in();
    void zoom_out();
    void fit_width(double width);
    void fit_height(double height);

    bool is_on_screen(Vector2 world_p);
}

#endif /* RENDER_H */
