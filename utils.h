#ifndef UTILS_H
#define UTILS_H

#include <SDL2/SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <string>
#include <vector>

/**
 * @brief Truncate a floating point value
 * @param n A floating point value
 * @param precision The precision of the truncature, i.e. 1000 for 3 digits after the point
 * @return A string containing the truncated value
 */
std::string truncate_to_string(double n, int precision);


class LTimer {
public:
    LTimer();
    virtual ~LTimer() {}

    void start();
    void stop();
    void pause();
    void unpause();

    Uint32 get_ticks() const;

    bool is_started() const { return m_started; }
    bool is_paused() const { return m_paused; }

private:
    Uint32 m_start_ticks;
    Uint32 m_paused_ticks;

    bool m_paused;
    bool m_started;
};


class LTexture {
public:
    LTexture(SDL_Renderer* renderer);
    LTexture(SDL_Renderer* renderer, SDL_Color color, TTF_Font* font);
    virtual ~LTexture();

    bool load_from_file(std::string filepath);
    bool load_from_rendered_text(std::string text, SDL_Color color, TTF_Font* font);
    bool load_from_rendered_text(std::string text);
    void free();
    void set_color(Uint8 r, Uint8 g, Uint8 b) {}
    void set_blend_mode(SDL_BlendMode blending) {}
    void set_alpha(Uint8 alpha) {}

    void render(int x, int y);

    int get_width() const { return m_width; }
    int get_height() const { return m_height; }

private:
    SDL_Texture* m_texture;
    SDL_Renderer* m_renderer;
    SDL_Color m_color;
    TTF_Font* m_font;
    int m_width;
    int m_height;
};


struct TextManager {
    TextManager(SDL_Renderer* renderer) : m_renderer(renderer) {}
    virtual ~TextManager() { free_all(); }

    void add_texture(SDL_Color color, TTF_Font* font, int x, int y);
    void render_all();
    void free_all();

    std::vector<LTexture*> textures;
    SDL_Renderer* m_renderer;
};

// Layout
struct VTextLayout : TextManager {
    VTextLayout(SDL_Renderer* renderer, int x, int y) : TextManager(renderer), m_x(x), m_y(y) {}
    virtual ~VTextLayout() { free_all(); }

    void add_texture(SDL_Color color, TTF_Font* font);

    /**
     * @brief Load texture with text and render it
     * @param row The row index of the texture in the layout, 1 being the first row (top)
     * @param text The string to load in the texture
     * @return true if the text is successfully loaded
     */
    bool load_text_and_render(size_t row, std::string text);

    // Top left corner of the layout
    int m_x;
    int m_y;
};


#endif /* UTILS_H */

