#include <iostream>
#include <vector>
#include "render.h"
#include "vector2.h"

namespace camera {
    static Vector2 position;
}

void render_point(SDL_Renderer* renderer, Vector2 p) {
    Vector2 px(camera::world_to_screen(p));
    SDL_RenderDrawPointF(renderer, px.x, px.y);
}

void render_line(SDL_Renderer* renderer, Vector2 p1, Vector2 p2) {
    Vector2 px1(camera::world_to_screen(p1));
    Vector2 px2(camera::world_to_screen(p2));
    SDL_RenderDrawLineF(renderer, px1.x, px1.y, px2.x, px2.y);
}

void render_filled_circle(SDL_Renderer *renderer, Vector2 center, double radius) {
    radius *= RENDER_SCALE;
    Vector2 center_px(camera::world_to_screen(center));
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
    Vector2 center_px(camera::world_to_screen(center));
    radius_ *= RENDER_SCALE;
    int x(center_px.x);
    int y(center_px.y);
    int radius(radius_);

    while (radius > 0) {
        // 70/99 is used as an approximation of 1/sqrt(2)
        const int approximation(radius * 8 * 70 / 99);
        const int points_qty((approximation + (8 - 1)) & -8);
        std::vector<SDL_Point> points(points_qty * 2);
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

        for (unsigned i(0); i < points.size(); ++i) {
            SDL_RenderDrawPoint(renderer, points[i].x, points[i].y);
        }
        --radius;
    }
}

void render_circle(SDL_Renderer* renderer, Vector2 center, double radius) {
    Vector2 center_px(camera::world_to_screen(center));
    radius *= RENDER_SCALE;
    double error(-radius);
    double x_(radius);
    double y_(0);
    double cx(center_px.x);
    double cy(center_px.y);

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
    Vector2 px(camera::world_to_screen(center));
    w *= RENDER_SCALE;
    h *= RENDER_SCALE;

    SDL_FRect rect{(float)px.x - w * 0.5, (float)px.y - h * 0.5, (float)w + 1, (float)h + 1};
    SDL_RenderDrawRectF(renderer, &rect);
}

Vector2 camera::world_to_screen(Vector2 world_p) {
    Vector2 screen_p;
    world_p -= camera::position;
    screen_p.x = world_p.x * RENDER_SCALE + SCREEN_WIDTH / 2.0;
    screen_p.y = -world_p.y * RENDER_SCALE + SCREEN_HEIGHT / 2.0;

    return screen_p;
}

Vector2 camera::screen_to_world(int px, int py) {
    Vector2 world_p;
    world_p.x = ((double)px - SCREEN_WIDTH / 2.0) / RENDER_SCALE;
    world_p.y = (SCREEN_HEIGHT / 2.0 - py) / RENDER_SCALE;
    world_p += camera::position;

    return world_p;
}

void camera::translate_screen_x(int dx) {
    camera::position.x += dx / RENDER_SCALE;
}

void camera::translate_screen_y(int dy) {
    camera::position.y += dy / RENDER_SCALE;
}

void camera::translate_world(Vector2 delta) {
    camera::position += delta;
}

void camera::zoom_in() {
    RENDER_SCALE *= 1.1;
}

void camera::zoom_out() {
    RENDER_SCALE /= 1.1;
}
