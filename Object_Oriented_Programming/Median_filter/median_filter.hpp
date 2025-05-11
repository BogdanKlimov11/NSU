#pragma once

#include "bitmap.hpp"
#include "neighbourhood.hpp"

class PixelMedianFilter {
public:
    virtual bmp::Pixel filter(Neighbourhood& notFiltered, size_t filtrationThreshold) = 0;
};

class RGBPixelMedianFilter : PixelMedianFilter{
public:
    bmp::Pixel filter(Neighbourhood& notFiltered, size_t filtrationThreshold) override;
};

class BlackWhitePixelMedianFilter : PixelMedianFilter{
public:
    bmp::Pixel filter(Neighbourhood& notFiltered, size_t filtrationThreshold) override;
};
