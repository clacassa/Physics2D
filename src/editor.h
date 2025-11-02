#ifndef EDITOR
#define EDITOR

#include <vector>
#include "rigid_body.h" // BodyType, RigidBodyDef
#include "shape.h"      // Shape
#include "link.h"       // Spring::DampingType
#include "render.h"
#include "vector2.h"

constexpr unsigned editor_ticks_default(50);
constexpr float spring_stiffness_default(0.5 * steel_density);
constexpr float spring_stiffness_infinite(1e4f * steel_density);

struct Control;

// Holds data related to interactive body creation
struct BodyCreator {
    RigidBodyDef body_def;
    Shape* body_shape;
    enum ShapeID { CIRCLE, RECTANGLE, POLYGON } shape_id = CIRCLE;
    std::vector<Vector2> points_set;
    unsigned points_count = 0;
};

// Holds data related to interactive spring creation
struct SpringCreator {
    float stiffness = spring_stiffness_default;
    Spring::DampingType damping_type = Spring::DampingType::UNDAMPED;
    bool incompressible = false;
};

class Editor {
public:
    typedef std::vector<std::vector<Vector2>> Grid;

    Editor(SDL_Renderer* renderer, double division);
    virtual ~Editor() {}

    void render(Control& control);
    
    // Return the node in window coordinates closest to the point p
    Vector2 track_point(Vector2 p);
    void update_grid();
    void compute_division();

    void on_mouse_left_click(Control& control);

    inline double get_div() const { return div; }
    inline Vector2 get_active_node() const { return active_node; }
    inline BodyType get_body_type() const { return body_creator.body_def.type; }
    inline bool get_enabled() const { return body_creator.body_def.enabled; }
    inline float get_stiffness() const { return spring_creator.stiffness; }
    inline Spring::DampingType get_damping() const { return spring_creator.damping_type; }

    inline BodyCreator get_body_creator() { return body_creator; }

    inline void toggle_help() { show_help_banner = !show_help_banner; }

private:
    double div;
    Grid m_grid;
    Vector2 active_node;

    // Rendering
    SDL_Renderer* m_renderer;
    std::string help;
    bool show_help_banner;

    // Creation tools
    BodyCreator body_creator;
    SpringCreator spring_creator;

    bool create_circle();
    bool create_rectangle();
    void create_polygon();

    void render_grid();
    void show_controls(bool* editor_active, Control& control);
    void render_body_creation(Control& control);
};

#endif /* EDITOR */
