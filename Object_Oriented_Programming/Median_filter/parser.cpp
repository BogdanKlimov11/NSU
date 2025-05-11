#include <vector>
#include <string>

#include "parser.hpp"
#include "exceptions.hpp"

using namespace std;

namespace {
    bool IsNumber(std::string& word) {
        try {
            size_t length;
            stoi(word, &length);
            return length == word.length();
        } catch (exception& exception) {
            return false;
        }
    }
}

inputData createInputData(const std::vector<std::string>& source) {
    inputData result;
    int size;
    int threshold;
    string originalFileName;
    string filteredFileName;
    int processedIntegers = 0;
    int processedStrings = 0;

    for (auto elem : source) {
        if (IsNumber(elem)) {
            switch (processedIntegers) {
                case 0:
                    size = stoi(elem);
                    break;
                case 1:
                    threshold = stoi(elem);
                    break;
                default:
                    break;
            }
            processedIntegers++;
        } else {
            switch (processedStrings) {
                case 0:
                    originalFileName = elem;
                    break;
                case 1:
                    filteredFileName = elem;
                    break;
                default:
                    break;
            }
            processedStrings++;
        }
    }

    if (size <= 1 || threshold < 0 || threshold > (size * size / 2)) {
        throw WrongFiltrationParamsExceptions();
    }
    if (processedStrings == 0 || processedIntegers == 0) {
        throw WrongInputExceptions();
    }

    result.originalFile = originalFileName;
    result.filteredFile = filteredFileName;
    result.neighbourhoodSize = size;
    result.filtrationThreshold = threshold;
    return result;
}

std::vector<std::string> readStream(istream& input) {
    vector<string> read;
    string line;

    while (getline(input, line)) {
        read.push_back(line);
    }
    return read;
}
