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

    unsigned trailing_zeros(0);
#if defined(__GNUC__)
    trailing_zeros = __builtin_ctz(precision);
#elif defined(_MSC_VER)
    trailing_zeros = _tzcnt_u32(precision);
#endif

    ss << std::setprecision(std::numeric_limits<double>::max_digits10 + trailing_zeros)
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
