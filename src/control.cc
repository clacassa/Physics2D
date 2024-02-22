#include "control.h"

Control::Simulation::Simulation()
:   running(false),
    slow_motion(false)
{}

Control::Editor::Editor()
:   active(false),
    adding_spring(false)
{}

Control::Control()
:   quit(false)
{}

Control::~Control() {
    simulation.running = false;
}