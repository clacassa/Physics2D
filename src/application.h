#ifndef APPLICATION_H
#define APPLICATION_H

#include <SDL.h>
#include <SDL_ttf.h>
#include "control.h"
#include "system_state.h"
#include "editor.h"
#include "settings.h"

class Application {
public:
    Application(SDL_Window* window, SDL_Renderer* renderer, double w, double h, TTF_Font* font);
    virtual ~Application();

    int run();
private:
    SDL_Window* m_window;
    SDL_Renderer* m_renderer;
    unsigned m_width;
    unsigned m_height;
    TTF_Font* m_font_main;
    SDL_Cursor* m_arrow_cursor;
    SDL_Cursor* m_crosshair_cursor;
    
    int m_exit_status;

    Settings m_settings;
    Control m_ctrl;
    SystemState m_world;
    Editor m_editor;

    double delta_time;

    void handle_event(SDL_Event& e, const double dt);

    // Demos
    void demo_collision();

    // GUI
    void show_menubar();
    void show_main_overlay(const float avg_fps);
    void show_placeholder_object();
    void show_property_editor(bool* p_open);
    void show_options_panel();
    void show_help_panel();
};  

#endif /* APPLICATION_H */
