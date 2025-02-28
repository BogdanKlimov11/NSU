#pragma once

#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>

#include "frequency_dictionary.hpp"
#include "parser.hpp"

class VectorDictionary : public FrequencyDictionary {
private:
    std::vector<std::pair<std::string, size_t>> Dictionary;
    std::vector<std::pair<std::string, size_t>>::iterator DictIterator;

public:
    VectorDictionary() = default;
    ~VectorDictionary() = default;

    void addWord(const std::string& word) override;
    bool ResetEnumerator() override;
    std::pair<std::string, size_t> getNext() override;
    bool IteratorIsEnd() override {
        return DictIterator == Dictionary.end();
    }

    std::vector<std::pair<std::string, size_t>>::iterator findWord(const std::string& word);
};
