#ifndef CONTROL_H
#define CONTROL_H

#include "vector2.h"

struct Control {
    struct Simulation {
        bool running;
        bool slow_motion;

        Simulation();
    };

    struct Editor {
        bool active;
        bool adding_spring;
        Vector2 cross_pointer;

        Editor();
    };

    struct Input {
        // Scene coordinates
        Vector2 pointer;
        Vector2 prev_click;

        Input() {}
    };

    Control();
    ~Control();

    bool quit;
    Simulation simulation;
    Editor editor;
    Input input;
};

#endif /* CONTROL_H */