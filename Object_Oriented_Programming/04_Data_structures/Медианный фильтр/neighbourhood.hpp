#pragma once

#include "bitmap.hpp"

class Neighbourhood {
private:
    size_t size;
    bmp::Pixel centralPixel;
    size_t centralPixelX;
    size_t centralPixelY;
    std::vector<bmp::Pixel> neighbouringPixels;

public:
    explicit Neighbourhood(size_t _size, const bmp::Pixel &_centralPixel, const std::vector<bmp::Pixel> &allPixelsOfImage, size_t _centralPixelX, size_t _centralPixelY, size_t imageWidth) noexcept(false);
    ~Neighbourhood() = default;

    [[nodiscard]] size_t getSize() const;
    [[nodiscard]] bmp::Pixel getCentralPixel() const;
    [[nodiscard]] std::vector<bmp::Pixel> getNeighbouringPixels() const;
};
