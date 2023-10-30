#include <iostream>
#include "utils.h"

LTimer::LTimer()
:   m_start_ticks(0), m_paused_ticks(0), m_paused(false), m_started(false) {}

void LTimer::start() {
    m_started = true;
    m_paused = false;

    m_start_ticks = SDL_GetTicks();
    m_paused_ticks = 0;
}

void LTimer::stop() {
    m_started = false;
    m_paused = false;

    m_start_ticks = 0;
    m_paused_ticks = 0;
}

void LTimer::pause() {
    if (m_started && !m_paused) {
        m_paused = true;
        m_paused_ticks = SDL_GetTicks() - m_start_ticks;
        m_start_ticks = 0;
    }
}

void LTimer::unpause() {
    if (m_started && m_paused) {
        m_paused = false;
        m_start_ticks = SDL_GetTicks() - m_paused_ticks;
        m_paused_ticks = 0;
    }
}

Uint32 LTimer::get_ticks() const {
    Uint32 time(0);
    if (m_started) {
        if (m_paused) {
            time = m_paused_ticks;
        }else {
            time = SDL_GetTicks() - m_start_ticks;
        }
    }

    return time;
}


LTexture::LTexture(SDL_Renderer* renderer)
:   m_texture(nullptr),
    m_renderer(renderer),
    m_width(0),
    m_height(0)
{}

LTexture::~LTexture() {
    free();
}

bool LTexture::load_from_file(std::string filepath) {
    free();

    SDL_Texture* new_texture(nullptr);
    SDL_Surface* loaded_surface(IMG_Load(filepath.c_str()));
    if (loaded_surface) {
        SDL_SetColorKey(loaded_surface, SDL_TRUE, SDL_MapRGB(loaded_surface->format,
                    0, 0xFF, 0xFF));
        new_texture = SDL_CreateTextureFromSurface(m_renderer, loaded_surface);
        if (new_texture) {
            m_width = loaded_surface->w;
            m_height = loaded_surface->h;
        }
        SDL_FreeSurface(loaded_surface);
    }

    m_texture = new_texture;
    return m_texture != nullptr;
}

bool LTexture::load_from_rendered_text(std::string text, SDL_Color color, TTF_Font* font) {
    free();

    SDL_Surface* text_surface(TTF_RenderText_Blended_Wrapped(font, text.c_str(), color, 500));
    if (text_surface) {
        m_texture = SDL_CreateTextureFromSurface(m_renderer, text_surface);
        if (m_texture) {
            m_width = text_surface->w;
            m_height = text_surface->h;
        }

        SDL_FreeSurface(text_surface);
    }

    return m_texture != nullptr;
}

void LTexture::free() {
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_width = 0;
        m_height = 0;
    }
}

void LTexture::render(int x, int y) {
    SDL_Rect render_quad({x, y, m_width, m_height});
    SDL_RenderCopy(m_renderer, m_texture, nullptr, &render_quad);
}

