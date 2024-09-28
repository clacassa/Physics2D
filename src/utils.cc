#include <iostream>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include "utils.h"
#include "render.h"
#include "config.h"

/* https://stackoverflow.com/questions/41294368/truncating-a-double-floating-point-at-a-certain-number-of-digits
*/
std::string truncate_to_string(double n, int precision) {
    std::stringstream ss;
    bool negative(n < 0);
    if (negative)
        n *= -1;
    double remainder((double) ((int)floor((n - floor(n)) * precision) % precision));
    if (negative && (floor(n) != 0 || remainder))
        ss << "-";
    ss << std::setprecision(std::numeric_limits<double>::max_digits10 + __builtin_ctz(precision))
       << floor(n);
    if (remainder) {
        ss << ".";
        for (int i(0); i < log10(precision) - (log10(remainder) + 1); ++i) {
            ss << "0";
        }
        ss << remainder;
    }
    return ss.str();
}

double deg2rad(const double deg_angle) {
    return deg_angle * PI / 180.0;
}

double rad2deg(const double rad_angle) {
    return rad_angle * 180.0 / PI;
}

Timer::Timer() {
    inv_frequency = 1.0 / SDL_GetPerformanceFrequency();
    start = SDL_GetPerformanceCounter();
    stop = 0;
}

void Timer::reset(bool halt) {
    start = SDL_GetPerformanceCounter();
    if (halt) {
        stop = start;
    }else {
        stop = 0;
    }
}

void Timer::halt() {
    stop = SDL_GetPerformanceCounter();
}

uint64_t Timer::get_ticks() {
    uint64_t ticks(SDL_GetPerformanceCounter());
    return ticks - start;
}

float Timer::get_seconds() {
    return get_elapsed();
}

float Timer::get_milliseconds() {
    return get_elapsed(1e3);
}

float Timer::get_microseconds() {
    return get_elapsed(1e6);
}

float Timer::get_elapsed(const double prescaler) {
    uint64_t count(SDL_GetPerformanceCounter());
    if (stop > 0) {
        count = stop;
    }
    return (float)(inv_frequency * (count - start) * prescaler);
}


LTexture::LTexture(SDL_Renderer* renderer)
:   m_texture(nullptr),
    m_renderer(renderer),
    m_font(nullptr),
    m_width(0),
    m_height(0)
{}

LTexture::LTexture(SDL_Renderer* renderer, SDL_Color color, TTF_Font* font)
:   m_texture(nullptr),
    m_renderer(renderer),
    m_color(color),
    m_font(font),
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

bool LTexture::load_from_rendered_text(std::string text) {
    free();
    SDL_Surface* text_surface(TTF_RenderText_Blended_Wrapped(m_font, text.c_str(), m_color, 0));
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

void TextManager::add_texture(SDL_Color color, TTF_Font* font, int x, int y) {}

void TextManager::render_all() {}

void TextManager::free_all() {
    for (auto t : textures) {
        t->free();
        delete t;
    }
    textures.clear();
}

VTextLayout::VTextLayout(SDL_Renderer* renderer, int x, int y)
:   TextManager(renderer),
    m_x(x),
    m_y(y),
    centered(false)
{}

VTextLayout::VTextLayout(SDL_Renderer* renderer, int y)
:   TextManager(renderer),
    m_x(0),
    m_y(y),
    centered(true)
{}

void VTextLayout::add_texture(SDL_Color color, TTF_Font* font) {
    textures.push_back(new LTexture(m_renderer, color, font));
}

bool VTextLayout::load_text_and_render(size_t row, std::string text) {
    if (!textures[row - 1]->load_from_rendered_text(text))
        return false;
    int y_offset(0);
    for (size_t i(0); i < row - 1; ++i) {
        y_offset += textures[i]->get_height();
    }
    if (centered)
        m_x = (SCREEN_WIDTH - textures[row - 1]->get_width()) * 0.5;
    textures[row - 1]->render(m_x, m_y + y_offset);
    return true;
}


void HTextLayout::add_texture(SDL_Color color, TTF_Font* font) {
    textures.push_back(new LTexture(m_renderer, color, font));
}

bool HTextLayout::load_text_and_render(size_t column, std::string text, int offset, bool center) {
    if (!textures[column - 1]->load_from_rendered_text(text))
        return false;
    int x_offset(offset);
    if (x_offset == 0) {
        for (size_t i(0); i < column - 1; ++i) {
            x_offset += textures[i]->get_width();
        }
    }else {
        if (center)
            x_offset -= textures[column - 1]->get_width() * 0.5;
    }

    textures[column - 1]->render(m_x + x_offset, m_y);
    return true;
}
