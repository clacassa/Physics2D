#ifndef UTILS_H
#define UTILS_H

#include <SDL2/SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <string>

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
    virtual ~LTexture();

    bool load_from_file(std::string filepath);
    bool load_from_rendered_text(std::string text, SDL_Color color, TTF_Font* font);
    void free();
    void set_color(Uint8 r, Uint8 g, Uint8 b);
    void set_blend_mode(SDL_BlendMode blending);
    void set_alpha(Uint8 alpha);

    void  render(int x, int y);

    int get_width() const { return m_width; }
    int get_height() const { return m_height; }

private:
    SDL_Texture* m_texture;
    SDL_Renderer* m_renderer;
    int m_width;
    int m_height;
};


class LTextManager {
public:
    LTextManager() {}
    virtual ~LTextManager() {}

    void load_and_render(SDL_Renderer* renderer, std::string text, SDL_Color color, TTF_Font* font) {}
};


#endif /* UTILS_H */

