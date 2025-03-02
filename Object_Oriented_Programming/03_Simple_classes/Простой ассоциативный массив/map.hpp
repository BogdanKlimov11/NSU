#pragma once

#include <cstring>

const size_t max_key_length = 256;

struct Pair {
    const char key[max_key_length]{};
    int value{};
};

class Map {
private:
    Pair* pairs;
    size_t current_size;

public:
    Map();
    ~Map();
    Map(const Map& ref_pair);
    Map& operator=(const Map&);
    size_t size() const;
    int& insert(const char* key);
    Pair* at(size_t index);
    Pair* find(const char* key);
    void erase(const char* key);
    const int& operator[](const char* key) const;
    int& operator[](const char* key);
protected:
};
