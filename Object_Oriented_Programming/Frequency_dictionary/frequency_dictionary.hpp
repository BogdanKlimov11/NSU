#pragma once

#include <iostream>
#include <fstream>

#include "parser.hpp"

class FrequencyDictionary {
public:
    virtual ~FrequencyDictionary() = default;

    virtual void addWord(const std::string& word) = 0;
    virtual bool ResetEnumerator() = 0;
    virtual std::pair<std::string, size_t> getNext() = 0;
    virtual bool IteratorIsEnd() = 0;
};
