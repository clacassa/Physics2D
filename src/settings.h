#ifndef SETTINGS_H
#define SETTINGS_H

struct Settings {
    bool slow_motion;
    bool draw_body_trajectory;
    bool highlight_collisions;
    bool draw_contact_points;
    bool draw_collision_normal;
    bool draw_bounding_boxes;
    bool draw_distance_proxys;

    Settings();
    void reset();
};

#endif /* SETTINGS_H */