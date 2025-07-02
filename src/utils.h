#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "vector2.h"
#include <string>

/**
 * @brief Truncate a floating point value
 * @param n A floating point value
 * @param precision The precision of the truncature, i.e. 1000 for 3 digits after the point
 * @return A string containing the truncated value
 */
std::string truncate_to_string(double n, int precision = 100);


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

namespace utils {
    struct Vec2f {
        float x;
        float y;
    };
}

// utility structure for realtime plot from ImPlot
struct ScrollingBuffer {


    unsigned max_size;
    int offset;
    std::vector<utils::Vec2f> data;

    ScrollingBuffer(int max_size_ = 2e3) {
        max_size = max_size_;
        offset  = 0;
        data.reserve(max_size);
    }

    inline void add_point(float x, float y) {
        if (data.size() < max_size) {
            data.push_back({x, y});
        }else {
            data[offset] = {x, y};
            offset =  (offset + 1) % max_size;
        }
    }
    inline void erase() {
        if (data.size() > 0) {
            data.clear();
            offset  = 0;
        }
    }
};

// utility structure for realtime plot from ImPlot
struct RollingBuffer {
    float span;
    std::vector<utils::Vec2f> data;

    RollingBuffer() {
        span = 2e3;
        data.reserve(span);
    }
    void add_point(float x, float y) {
        if (!data.empty() && data.size() >= span)
            data.clear();
        data.push_back({x, y});
    }
};


#endif /* UTILS_H */

