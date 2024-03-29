#include <iostream>
#include <vector>
#include "render.h"
#include "vector2.h"


namespace camera {
    static Vector2 position = {SCENE_WIDTH * 0.5, SCENE_HEIGHT * 0.5};
}

void render_point(SDL_Renderer* renderer, Vector2 p) {
    Vector2 px(camera::transform_world_to_screen(p));
    SDL_RenderDrawPointF(renderer, px.x, px.y);
}

void render_line(SDL_Renderer* renderer, Vector2 p1, Vector2 p2) {
    Vector2 px1(camera::transform_world_to_screen(p1));
    Vector2 px2(camera::transform_world_to_screen(p2));
    SDL_RenderDrawLineF(renderer, px1.x, px1.y, px2.x, px2.y);
}

void render_filled_circle(SDL_Renderer *renderer, Vector2 center, double radius) {
    radius *= RENDER_SCALE;
    Vector2 center_px(camera::transform_world_to_screen(center));
    radius = (int)radius;

    for (int w(0); w < radius * 2; ++w) {
        for (int h(0); h < radius * 2; ++h) {
            int dx(radius - w); // horizontal offset
            int dy(radius - h); // vertical offset
            if ((dx*dx + dy*dy) <= (radius * radius))
                SDL_RenderDrawPointF(renderer, center_px.x + dx, center_px.y + dy);
        }
    }
}


void render_fill_circle_fast(SDL_Renderer * renderer, Vector2 center, double radius_) {
    Vector2 center_px(camera::transform_world_to_screen(center));
    radius_ *= RENDER_SCALE;
    int x(center_px.x);
    int y(center_px.y);
    int radius(radius_);

    while (radius > 0) {
        // 35 / 49 is a slightly biased approximation of 1/sqrt(2)
        const int approximation(radius * 8 * 35 / 49);
        const int points_qty((approximation + (8 - 1)) & -8);
        SDL_Point points[points_qty];
        int draw_count(0);

        const int32_t diameter((radius * 2));

        int32_t dx(radius - 1);
        int32_t dy(0);
        int32_t tx(1);
        int32_t ty(1);
        int32_t error(tx - diameter);

        while( dx >= dy )
        {
            // Each of the following renders an octant of the circle
            points[draw_count+0] = { x + dx, y - dy };
            points[draw_count+1] = { x + dx, y + dy };
            points[draw_count+2] = { x - dx, y - dy };
            points[draw_count+3] = { x - dx, y + dy };
            points[draw_count+4] = { x + dy, y - dx };
            points[draw_count+5] = { x + dy, y + dx };
            points[draw_count+6] = { x - dy, y - dx };
            points[draw_count+7] = { x - dy, y + dx };

            draw_count += 8;

            if( error <= 0 )
            {
                ++dy;
                error += ty;
                ty += 2;
            }

            if( error > 0 )
            {
                --dx;
                tx += 2;
                error += (tx - diameter);
            }
        }

        SDL_RenderDrawPoints(renderer, points, draw_count);
        --radius;
    }
}

void render_circle(SDL_Renderer* renderer, Vector2 center, double radius) {
    Vector2 center_px(camera::transform_world_to_screen(center));
    radius *= RENDER_SCALE;

    double error(-radius);
    double x_(radius - 0.5);
    double y_(0.5);
    double cx(center_px.x - 0.5);
    double cy(center_px.y - 0.5);

    while (x_ >= y_) {
        SDL_RenderDrawPoint(renderer, (int)(cx + x_), (int)(cy + y_));
        SDL_RenderDrawPoint(renderer, (int)(cx + y_), (int)(cy + x_));
        if (x_ != 0) {
            SDL_RenderDrawPoint(renderer, (int)(cx - x_), (int)(cy + y_));
            SDL_RenderDrawPoint(renderer, (int)(cx + y_), (int)(cy - x_));
        }
        if (y_ != 0) {
            SDL_RenderDrawPoint(renderer, (int)(cx + x_), (int)(cy - y_));
            SDL_RenderDrawPoint(renderer, (int)(cx - y_), (int)(cy + x_));
        }
        if (x_ != 0 && y_ != 0) {
            SDL_RenderDrawPoint(renderer, (int)(cx - x_), (int)(cy - y_));
            SDL_RenderDrawPoint(renderer, (int)(cx - y_), (int)(cy - x_));
        }

        error += y_;
        ++y_;
        error += y_;

        if (error >= 0) {
            --x_;
            error -= x_;
            error -= x_;
        }
    }
}

void render_rectangle(SDL_Renderer* renderer, Vector2 center, double w, double h) {
    Vector2 px(camera::transform_world_to_screen(center));
    w *= RENDER_SCALE;
    h *= RENDER_SCALE;

    SDL_FRect rect{(float)px.x, (float)px.y, (float)w, (float)h};
    SDL_RenderDrawRectF(renderer, &rect);
}

Vector2 camera::transform_world_to_screen(Vector2 world_p) {
    Vector2 screen_p;
    world_p -= camera::position;
    screen_p.x = world_p.x * RENDER_SCALE + SCREEN_WIDTH / 2.0;
    screen_p.y = -world_p.y * RENDER_SCALE + SCREEN_HEIGHT / 2.0;

    return screen_p;
}

Vector2 camera::transform_screen_to_world(int px, int py) {
    Vector2 world_p;
    world_p.x = ((double)px - SCREEN_WIDTH / 2.0) / RENDER_SCALE;
    world_p.y = (SCREEN_HEIGHT / 2.0 - py) / RENDER_SCALE;
    world_p += camera::position;

    return world_p;
}

void camera::translate_left() {
    camera::position.x -= 50 / RENDER_SCALE;
}

void camera::translate_down() {
    camera::position.y -= 50 / RENDER_SCALE;
}

void camera::translate_right() {
    camera::position.x += 50 / RENDER_SCALE;
}

void camera::translate_up() {
    camera::position.y += 50 / RENDER_SCALE;
}
