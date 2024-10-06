#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL.h>
#include "control.h"
#include "world.h"
#include "editor.h"
#include "settings.h"

class Application {
public:
    Application(SDL_Window* window, SDL_Renderer* renderer, double w, double h);
    virtual ~Application();

    int run();
private:
    SDL_Window* m_window;
    SDL_Renderer* m_renderer;
    unsigned m_width;
    unsigned m_height;
    SDL_Cursor* m_arrow_cursor;
    SDL_Cursor* m_crosshair_cursor;
    
    int m_exit_status;

    Settings m_settings;
    Control m_ctrl;
    World m_world;
    Editor m_editor;

    double frame_time;
    double time_step;

    // Events
    void parse_event(SDL_Event& event);
    void parse_keybd_event(SDL_Event& keybd_event);
    void parse_mouse_button_event(SDL_Event& mouse_event);
    void parse_mouse_motion_event(SDL_Event& motion_event);
    void parse_mouse_wheel_event(SDL_Event& wheel_event);

    // Demos
    void demo_collision();
    void demo_stacking();

    // GUI
    void show_menubar();
    void show_main_overlay(const float avg_fps);
    void show_placeholder_object();
    void show_property_editor(bool* p_open);
    void show_settings_panel();
    void show_help_panel();
};  

#endif /* APPLICATION_H */
