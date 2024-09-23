#include "settings.h"
#include "config.h"

Settings::Settings() {
    reset();
}

void Settings::reset() {
    slow_motion = 0;
    draw_body_trajectory = 0;
#ifdef DEBUG
    highlight_collisions = 1;
    draw_contact_points = 1;
    draw_collision_normal = 1;
    draw_bounding_boxes = 1;
    draw_distance_proxys = 1;
#else
    highlight_collisions = 0;
    draw_contact_points = 0;
    draw_collision_normal = 0;
    draw_bounding_boxes = 0;
    draw_distance_proxys = 0;
#endif
}