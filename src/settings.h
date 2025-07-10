#ifndef SETTINGS_H
#define SETTINGS_H

struct Settings {
    bool slow_motion;
    bool enable_gravity;
    bool draw_body_trajectory;
    bool draw_center_of_mass;
    bool highlight_collisions;
    bool draw_contact_points;
    bool draw_collision_normal;
    bool draw_bounding_boxes;
    bool draw_distance_proxys;
    bool plot_position;
    bool plot_velocity;
    bool plot_phase_plane;

    Settings();
    void reset();
};

#endif /* SETTINGS_H */
