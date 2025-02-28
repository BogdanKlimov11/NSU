#pragma once

#include <sstream>
#include <iostream>
#include <string>
#include <cctype>
#include <vector>

#include "frequency_dictionary.hpp"

class FrequencyDictionary;

class Parser final {
private:
    std::istream& is_;

public:
    Parser(std::istream& is) : is_(is) {}
    std::pair<std::vector<std::string>, bool> getWord();
    std::vector<std::string> filterPunctuatuion(const std::string& str);
    void fillDictionary(FrequencyDictionary* dict);
};
