#include "neighbourhood.hpp"
#include "exceptions.hpp"
#include "vector_index.hpp"

using namespace std;
using namespace bmp;

Neighbourhood::Neighbourhood(size_t _size, const bmp::Pixel &_centralPixel, const std::vector<bmp::Pixel> &allPixelsOfImage, size_t _centralPixelX, size_t _centralPixelY, size_t imageWidth) {
    if (_size <= 0) {
        throw WrongFiltrationParamsExceptions();
    }
    size = _size;
    centralPixel = _centralPixel;
    centralPixelX = _centralPixelX;
    centralPixelY = _centralPixelY;
    auto centralPixelIdx = transform2Dinto1D(centralPixelX, centralPixelY, imageWidth);
    vector<Pixel> neighbourhood(size * size);
    for (int idx = 0; idx < size * size; idx++) {
        neighbourhood[idx] = allPixelsOfImage[centralPixelIdx - (size * (size / 2) + (size / 2)) + idx];
    }
    neighbouringPixels = neighbourhood;
}

size_t Neighbourhood::getSize() const {
    return size;
}

bmp::Pixel Neighbourhood::getCentralPixel() const {
    return centralPixel;
}

std::vector<bmp::Pixel> Neighbourhood::getNeighbouringPixels() const {
    return neighbouringPixels;
}
