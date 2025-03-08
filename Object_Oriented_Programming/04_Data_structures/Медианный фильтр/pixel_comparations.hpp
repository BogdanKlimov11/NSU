#pragma once

#include "bitmap.hpp"

inline bool pixelCmpRed(const bmp::Pixel& first, const bmp::Pixel& second) {
    return first.r < second.r;
}

inline bool pixelCmpBlue(const bmp::Pixel& first, const bmp::Pixel& second) {
    return first.b < second.b;
}

inline bool pixelCmpGreen(const bmp::Pixel& first, const bmp::Pixel& second) {
    return first.g < second.g;
}

struct compareWithControlPixel {
    const bmp::Pixel& _this;

    explicit compareWithControlPixel(const bmp::Pixel& controlPixel) : _this(controlPixel) {}

    inline bool operator()(const bmp::Pixel& that) const {
        return _this.r == that.r && _this.b == that.b && _this.g == that.g;
    }
};
