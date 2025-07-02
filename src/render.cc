#include <SDL2_gfxPrimitives.h>
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

void render_circle(SDL_Renderer* renderer, Vector2 center, double radius) {
    Vector2 center_px(camera::world_to_screen(center));
    int x0(center_px.x);
    int y0(center_px.y);
    int rad(radius * RENDER_SCALE);
    int f(1 - rad);
    int dx(0);
    int dy(-2 * rad);
    int x(0);
    int y(rad);

    SDL_Point initial_points[4];
    initial_points[0] = {x0, y0 + rad};
    initial_points[1] = {x0, y0 - rad};
    initial_points[2] = {x0 + rad, y0};
    initial_points[3] = {x0 - rad, y0};
    SDL_RenderDrawPoints(renderer, initial_points, 4);

    while (x < y) {
        if (f >= 0) {
            --y;
            dy += 2;
            f += dy;
        }

        ++x;
        dx += 2;
        f += dx + 1;

        SDL_Point points[8];
        points[0] = {x0 + x, y0 + y};
        points[1] = {x0 - x, y0 + y};
        points[2] = {x0 + x, y0 - y};
        points[3] = {x0 - x, y0 - y};
        points[4] = {x0 + y, y0 + x};
        points[5] = {x0 - y, y0 + x};
        points[6] = {x0 + y, y0 - x};
        points[7] = {x0 - y, y0 - x};
        SDL_RenderDrawPoints(renderer, points, 8);
    }
}

void render_circle_fill(SDL_Renderer * renderer, Vector2 center, double radius_) {
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

void render_circle_fill_raster(SDL_Renderer* renderer, Vector2 center, double radius) {
    Vector2 center_px(camera::world_to_screen(center));
    int x0(center_px.x);
    int y0(center_px.y);
    int rad(radius * RENDER_SCALE);
    int f(1 - rad);
    int dx(0);
    int dy(-2 * rad);
    int x(0);
    int y(rad);

    SDL_RenderDrawLine(renderer, x0 - rad, y0, x0 + rad, y0);

    int old_x(0);
    int old_y(0);
    while (x < y) {
        if (f >= 0) {
            --y;
            dy += 2;
            f += dy;
        }

        ++x;
        dx += 2;
        f += dx + 1;

        if (y != old_y) {
            SDL_RenderDrawLine(renderer, x0 - x, y0 + y, x0 + x, y0 + y);
            SDL_RenderDrawLine(renderer, x0 - x, y0 - y, x0 + x, y0 - y);
        }

        if (x != old_x) {
            SDL_RenderDrawLine(renderer, x0 - y, y0 + x, x0 + y, y0 + x);
            SDL_RenderDrawLine(renderer, x0 - y, y0 - x, x0 + y, y0 - x);
        }

        old_x = x; 
        old_y = y;
    }
}

void render_rectangle(SDL_Renderer* renderer, Vector2 center, float w, float h) {
    Vector2 px(camera::world_to_screen(center));
    w *= RENDER_SCALE;
    h *= RENDER_SCALE;

    SDL_FRect rect{(float)px.x - w * 0.5f, (float)px.y - h * 0.5f, w + 1, h + 1};
    SDL_RenderDrawRectF(renderer, &rect);
}

void render_polygon_fill(SDL_Renderer* renderer, Vector2* vertices, uint8_t n, uint32_t color) {
    Sint16 vx[n];
    Sint16 vy[n];
    for (uint8_t i (0); i < n; ++i) {
        vertices[i] = camera::world_to_screen(vertices[i]);
        vx[i] = vertices[i].x;
        vy[i] = vertices[i].y;
    }

    filledPolygonColor(renderer, vx, vy, n, color);
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

bool camera::is_on_screen(Vector2 world_p) {
    Vector2 p(camera::world_to_screen(world_p));
    return p.x >= 0 && p.x < SCREEN_WIDTH && p.y >= 0 && p.y < SCREEN_HEIGHT;
}
