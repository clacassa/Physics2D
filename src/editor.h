#ifndef EDITOR
#define EDITOR

#include <vector>
#include "rigid_body.h" // BodyType
#include "link.h"       // Spring::DampingType
#include "render.h"
#include "vector2.h"
#include "utils.h"      // Texture, VTextLayout

//extern const unsigned SCREEN_WIDTH;

constexpr float spring_stiffness_default(0.5 * steel_density);
constexpr float spring_stiffness_infinite(1e4f * steel_density);

class Editor {
public:
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
    inline BodyType get_body_type() const { return body_type; }
    inline bool get_enabled() const { return body_enabled; }
    inline float get_stiffness() const { return spring_stiffness; }
    inline Spring::DampingType get_damping() const { return spring_damping; }

    inline void toggle_help() { show_help_banner = !show_help_banner; }
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
    BodyType body_type;
    bool body_enabled;

    bool spring_incompressible;
    float spring_stiffness;
    Spring::DampingType spring_damping;

    bool show_help_banner;

    void render_grid();
};

#endif /* EDITOR */
