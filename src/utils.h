#ifndef UTILS_H
#define UTILS_H

#include <SDL_render.h>
#include <SDL_timer.h>
#include <string>
#include <vector>

/**
 * @brief Truncate a floating point value
 * @param n A floating point value
 * @param precision The precision of the truncature, i.e. 1000 for 3 digits after the point
 * @return A string containing the truncated value
 */
std::string truncate_to_string(double n, int precision = 100);

double deg2rad(const double deg_angle);
double rad2deg(const double rad_angle);


class Timer {
public:
    Timer();

    void reset(bool halt = false);
    void halt();
    uint64_t get_ticks();
    float get_seconds();
    float get_milliseconds();
    float get_microseconds();
private:
    double inv_frequency;
    uint64_t start;
    uint64_t stop;

    float get_elapsed(const double prescaler = 1.0);
};

#endif /* UTILS_H */

