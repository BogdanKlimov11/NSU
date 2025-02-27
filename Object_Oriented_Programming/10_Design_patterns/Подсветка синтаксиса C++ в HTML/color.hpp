#pragma once

#include <random>
#include <iomanip>
#include <iostream>

struct Color {
    int r{255};
    int g{255};
    int b{255};
    static Color generate() {
        Color color{};
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dist(0, 255);
        while ((color.r + color.g + color.b) / 3 > 130 || (color.r + color.g + color.b) / 3 < 70) {
            color = {dist(gen), dist(gen), dist(gen)};
        }
        return color;
    }
};

inline std::ostream& operator<<(std::ostream& out, const Color& c) {
    out << std::hex << std::setfill('0')
        << std::setw(2) << c.r
        << std::setw(2) << c.g
        << std::setw(2) << c.b;
    return out;
}
