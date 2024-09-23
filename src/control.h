#ifndef CONTROL_H
#define CONTROL_H

#include "vector2.h"

struct Control {
    struct Simulation {
        bool running = false;
    };

    struct Editor {
        bool active = false;
        bool adding_spring = false;
    };

    struct Input {
        // Scene coordinates
        Vector2 pointer;
        Vector2 prev_click;
    };

    bool quit = false;
    Simulation simulation;
    Editor editor;
    Input input;

    ~Control() { simulation.running = false; }
};

#endif /* CONTROL_H */