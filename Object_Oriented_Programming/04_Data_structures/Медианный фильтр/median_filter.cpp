#include <map>

#include "median_filter.hpp"
#include "vector_index.hpp"
#include "pixel_comparations.hpp"

using namespace bmp;
using namespace std;

namespace {
    vector<Pixel> redFilter(const vector<Pixel>& notFiltered, const size_t threshold, const Pixel& filteringPixel) {
        vector<Pixel> processing = notFiltered;
        sort(processing.begin(), processing.end(), pixelCmpRed);
        auto centralPixelIdx = vectorIndex(processing, filteringPixel);
        auto centralPixel = processing[centralPixelIdx];
        auto medianIdx = (processing.size() - 1) / 2;
        auto medianIterator = processing.begin();
        advance(medianIterator, medianIdx);
        auto median = processing[medianIdx];
        int maxRedBrightness = 0;
        int minRedBrightness = 255;

        for (auto it = medianIterator - threshold; it < medianIterator + threshold; it++) {
            if (maxRedBrightness < (*it).r) {
                maxRedBrightness = (*it).r;
            }
            if (minRedBrightness > (*it).r) {
                minRedBrightness = (*it).r;
            }
        }

        if (centralPixel.r > maxRedBrightness || centralPixel.r < minRedBrightness) {
            auto newCentralPixel = Pixel(median.r, centralPixel.g, centralPixel.b);
            vector<Pixel> filtered = notFiltered;
            filtered[vectorIndex(filtered, filteringPixel)] = newCentralPixel;
            return filtered;
        }
        return notFiltered;
    }

    vector<Pixel> greenFilter(const vector<Pixel>& notFiltered, const size_t threshold, const Pixel& filteringPixel) {
        vector<Pixel> processing = notFiltered;
        sort(processing.begin(), processing.end(), pixelCmpGreen);
        auto centralPixelIdx = vectorIndex(processing, filteringPixel);
        auto centralPixel = processing[centralPixelIdx];
        auto medianIdx = (processing.size() - 1) / 2;
        auto medianIterator = processing.begin();
        advance(medianIterator, medianIdx);
        auto median = processing[medianIdx];
        int maxGreenBrightness = 0;
        int minGreenBrightness = 255;

        for (auto it = medianIterator - threshold; it < medianIterator + threshold; it++) {
            if (maxGreenBrightness < (*it).g) {
                maxGreenBrightness = (*it).g;
            }
            if (minGreenBrightness > (*it).g) {
                minGreenBrightness = (*it).g;
            }
        }

        if (centralPixel.g > maxGreenBrightness || centralPixel.g < minGreenBrightness) {
            auto newCentralPixel = Pixel(centralPixel.r, median.g, centralPixel.b);
            vector<Pixel> filtered = notFiltered;
            filtered[vectorIndex(filtered, filteringPixel)] = newCentralPixel;
            return filtered;
        }
        return notFiltered;
    }

    vector<Pixel> blueFilter(const vector<Pixel>& notFiltered, const size_t threshold, const Pixel& filteringPixel) {
        vector<Pixel> processing = notFiltered;
        sort(processing.begin(), processing.end(), pixelCmpBlue);
        auto centralPixelIdx = vectorIndex(processing, filteringPixel);
        auto centralPixel = processing[centralPixelIdx];
        auto medianIdx = (processing.size() - 1) / 2;
        auto medianIterator = processing.begin();
        advance(medianIterator, medianIdx);
        auto median = processing[medianIdx];
        int maxBlueBrightness = 0;
        int minBlueBrightness = 255;

        for (auto it = medianIterator - threshold; it < medianIterator + threshold; it++) {
            if (maxBlueBrightness < (*it).b) {
                maxBlueBrightness = (*it).b;
            }
            if (minBlueBrightness > (*it).b) {
                minBlueBrightness = (*it).b;
            }
        }

        if (centralPixel.b > maxBlueBrightness || centralPixel.b < minBlueBrightness) {
            auto newCentralPixel = Pixel(centralPixel.r, centralPixel.g, median.b);
            vector<Pixel> filtered = notFiltered;
            filtered[vectorIndex(filtered, filteringPixel)] = newCentralPixel;
            return filtered;
        }
        return notFiltered;
    }
}

Pixel RGBPixelMedianFilter::filter(Neighbourhood& notFiltered, size_t filtrationThreshold) {
    auto central
