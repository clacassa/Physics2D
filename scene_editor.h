#ifndef SCENE_EDITOR
#define SCENE_EDITOR

#include <vector>
#include "vector2.h"
#include "render.h"

typedef std::vector<std::vector<Vector2>> Grid;

class SceneEditor {
public:
    SceneEditor(double division);
    virtual ~SceneEditor() {}

    void render_grid(SDL_Renderer* renderer);
    // Return the node in window coordinates closest to the point
    Vector2 track_point(Vector2 p);
private:
    size_t n;
    Grid m_grid;
    Vector2 active_node;
};

#endif /* SCENE_EDITOR */