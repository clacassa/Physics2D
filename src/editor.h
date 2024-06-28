#ifndef EDITOR
#define EDITOR

#include <vector>
#include "rigid_body.h" // BodyType
#include "link.h"       // Spring::DampingType
#include "render.h"
#include "vector2.h"
#include "utils.h"      // Texture, VTextLayout

//extern const unsigned SCREEN_WIDTH;

constexpr float spring_default_stiffness(0.5 * stl_steel_density);
constexpr float spring_infinite_stiffness(1e4f * stl_steel_density);

class Editor {
public:
    enum BodyShape { BALL = 0, RECTANGLE };

    typedef std::vector<std::vector<Vector2>> Quadrant;

    Editor(SDL_Renderer* renderer, TTF_Font* font, double division);
    virtual ~Editor() {}

    void render();
    void show_controls(bool* editor_active);
    // Return the node in window coordinates closest to the point p
    Vector2 track_point(Vector2 p);
    void update_grid();

    inline double get_div() const { return div; }
    inline Vector2 get_active_node() const { return active_node; }
    inline BodyShape get_body_shape() const { return body_shape; }
    inline BodyType get_body_type() const { return body_type; }
    inline bool get_enabled() const { return enabled_body; }
    inline bool get_show_help() const { return show_help_banner; }
    inline float get_stiffness() const { return current_stiffness; }
    inline Spring::DampingType get_damping() const { return damping; }

    inline void set_body_shape(const BodyShape shape) { body_shape = shape; }
    inline void toggle_movable(const BodyType type) { body_type = type; }
    inline void toggle_enabled() { enabled_body = !enabled_body; }
    inline void toggle_help() { show_help_banner = !show_help_banner; }
    inline void set_spring_damping(const Spring::DampingType damping) { this->damping = damping; }
private:
    double div;
    Quadrant m_first_quad, m_second_quad, m_third_quad, m_fourth_quad;
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
