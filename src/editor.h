#ifndef EDITOR
#define EDITOR

#include <vector>
#include "link.h"
#include "vector2.h"
#include "render.h"
#include "utils.h"

class Editor {
public:
    const double DIV_MIN = SCENE_WIDTH / 100;
    const double DIV_MAX = SCENE_WIDTH / 20;

    enum BodyType { BALL, RECTANGLE };

    typedef std::vector<std::vector<Vector2>> Grid;

    Editor(SDL_Renderer* renderer, TTF_Font* font, double division);
    virtual ~Editor() {}

    void render();
    // Return the node in window coordinates closest to the point
    Vector2 track_point(Vector2 p);
    void increase_div();
    void decrease_div();
    void update_grid();

    double get_div() const { return this->div; }
    bool get_movable() const { return movable_body; }
    bool get_enabled() const { return enabled_body; }
    bool get_show_help() const { return show_help_banner; }
    bool get_stiff_spring() const { return very_stiff_spring; }
    BodyType get_body_type() const { return body_type; }
    Spring::DampingType get_damping() const { return damping; }

    void toggle_movable() { movable_body = !movable_body; }
    void toggle_enabled() { enabled_body = !enabled_body; }
    void toggle_help() { show_help_banner = !show_help_banner; }
    void toggle_stiff_spring() { very_stiff_spring = !very_stiff_spring; }
    void set_body_type(const BodyType type) { body_type = type; }
    void set_spring_damping(const Spring::DampingType damping) { this->damping = damping; }
private:
    double div;
    size_t n;
    Grid m_grid;
    Vector2 active_node;

    // Rendering
    SDL_Renderer* m_renderer;
    TTF_Font* m_font;
    LTexture m_title;
    VTextLayout m_banner;
    std::string help;

    // Control
    bool movable_body;
    bool enabled_body;
    bool show_help_banner;
    bool very_stiff_spring;
    BodyType body_type;
    Spring::DampingType damping;

    void render_grid();
    void update_help();
};

#endif /* EDITOR */