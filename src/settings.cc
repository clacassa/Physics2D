#include "settings.h"
#include "config.h"

Settings::Settings() {
    reset();
}

void Settings::reset() {
    slow_motion = 0;
    draw_body_trajectory = 0;
    draw_center_of_mass = 0;
#ifdef DEBUG
    highlight_collisions = 0;
    draw_contact_points = 0;
    draw_collision_normal = 0;
    draw_bounding_boxes = 0;
    draw_distance_proxys = 0;
#else
    highlight_collisions = 0;
    draw_contact_points = 0;
    draw_collision_normal = 0;
    draw_bounding_boxes = 0;
    draw_distance_proxys = 0;
#endif
    enable_gravity = 1;
    plot_position = 0;
    plot_velocity = 0;
    plot_phase_plane = 0;
}
