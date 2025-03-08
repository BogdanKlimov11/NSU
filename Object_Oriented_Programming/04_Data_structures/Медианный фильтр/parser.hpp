#pragma once

#include <iostream>
#include <vector>
#include <string>

struct inputData {
    std::string originalFile;
    std::string filteredFile;
    size_t neighbourhoodSize;
    size_t filtrationThreshold;
};

std::vector<std::string> readStream(std::istream &input);
inputData createInputData(const std::vector<std::string>& source);
