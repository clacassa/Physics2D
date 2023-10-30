#include <iostream>
#include <vector>
#include "render.h"

void render_body_circle(SDL_Renderer *renderer, double x, double y, double radius) {
    radius *= RENDER_SCALE;
    x *= RENDER_SCALE;
    y = (SCENE_HEIGHT - y) * RENDER_SCALE;

    radius = (int)radius;
    x = (int)x;
    y = (int)y;

    for (int w(0); w < radius * 2; ++w) {
        for (int h(0); h < radius * 2; ++h) {
            int dx(radius - w); // horizontal offset
            int dy(radius - h); // vertical offset
            if ((dx*dx + dy*dy) <= (radius * radius))
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
        }
    }
}

void render_line(SDL_Renderer* renderer, double x1, double y1, double x2, double y2) {
    x1 *= RENDER_SCALE;
    y1 = (SCENE_HEIGHT - y1) * RENDER_SCALE;
    x2 *= RENDER_SCALE;
    y2 = (SCENE_HEIGHT - y2) * RENDER_SCALE;
    SDL_RenderDrawLineF(renderer, x1, y1, x2, y2);
}

void render_circle(SDL_Renderer * renderer, double x_, double y_, double radius_) {
    x_ *= RENDER_SCALE;
    y_ = (SCENE_HEIGHT - y_) * RENDER_SCALE;
    radius_ *= RENDER_SCALE;
    int x(x_);
    int y(y_);
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

