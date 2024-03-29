#ifndef EDITOR
#define EDITOR

#include <vector>
#include "rigid_body.h" // BodyType
#include "link.h"       // Spring::DampingType
#include "render.h"
#include "vector2.h"
#include "utils.h"      // Texture, VTextLayout

//extern const unsigned SCREEN_WIDTH;

constexpr float spring_default_stiffness(25.0f);
constexpr float spring_infnite_stiffness(1e6f);

class Editor {
public:
    const double DIV_MIN = SCENE_WIDTH / 100;
    const double DIV_MAX = SCENE_WIDTH / 20;

    enum BodyShape { BALL = 0, RECTANGLE };

    typedef std::vector<std::vector<Vector2>> Grid;

    Editor(SDL_Renderer* renderer, TTF_Font* font, double division);
    virtual ~Editor() {}

    void render();
    void imgui_controls(bool* editor_active);
    // Return the node in window coordinates closest to the point p
    Vector2 track_point(Vector2 p);
    void increase_div();
    void decrease_div();
    void update_grid();

    inline double get_div() const { return div; }
    inline BodyShape get_body_shape() const { return body_shape; }
    inline BodyType get_body_type() const { return body_type; }
    inline bool get_enabled() const { return enabled_body; }
    inline bool get_show_help() const { return show_help_banner; }
    // inline bool get_stiff_spring() const { return spring_very_stiff; }
    inline float get_stiffness() const { return current_stiffness; }
    inline Spring::DampingType get_damping() const { return damping; }

    inline void set_body_shape(const BodyShape shape) { body_shape = shape; }
    inline void toggle_movable(const BodyType type) { body_type = type; }
    inline void toggle_enabled() { enabled_body = !enabled_body; }
    inline void toggle_help() { show_help_banner = !show_help_banner; }
    inline void set_spring_damping(const Spring::DampingType damping) { this->damping = damping; }
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
    BodyShape body_shape;
    BodyType body_type;
    bool enabled_body;
    bool show_help_banner;
    bool spring_very_stiff;
    float current_stiffness;
    Spring::DampingType damping;

    void render_grid();
    void update_help();
};

#endif /* EDITOR */
