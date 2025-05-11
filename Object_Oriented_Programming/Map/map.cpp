#include <iostream>
#include <cstring>

#include "map.hpp"

using namespace std;

Map::Map() : pairs(nullptr), current_size(0) {}

Map::~Map() {
    delete[] pairs;
}

Map::Map(const Map& val) {
    pairs = new Pair[val.current_size];
    current_size = val.current_size;
    memcpy(pairs, val.pairs, sizeof(Pair) * current_size);
}

Map& Map::operator=(const Map& val) {
    if (this == &val) {
        return *this;
    }
    delete[] pairs;
    pairs = new Pair[val.current_size];
    current_size = val.current_size;
    memcpy(pairs, val.pairs, sizeof(Pair) * current_size);
    return *this;
}

size_t Map::size() const {
    return current_size;
}

int& Map::insert(const char* key) {
    if (key == nullptr || key[0] == 0) {
        throw std::invalid_argument("invalid key");
    }
    if (this->find(key) != nullptr) {
        return this->find(key)->value;
    }
    Pair* new_pairs = new Pair[current_size + 1];
    memcpy(new_pairs, pairs, sizeof(Pair) * current_size);
    strncpy_s((char*)new_pairs[current_size].key, max_key_length, (char*)key, _TRUNCATE);
    new_pairs[current_size].value = 0;
    delete[] pairs;
    pairs = new_pairs;
    current_size += 1;
    return pairs[current_size - 1].value;
}

Pair* Map::at(size_t index) {
    if (index >= current_size) {
        return nullptr;
    }
    return &pairs[index];
}

Pair* Map::find(const char* key) {
    for (size_t i = 0; i < current_size; i++) {
        if (strncmp(pairs[i].key, key, max_key_length - 1) == 0) {
            return &pairs[i];
        }
    }
    return nullptr;
}

void Map::erase(const char* key) {
    if (key == nullptr || key[0] == 0) {
        throw std::invalid_argument("invalid key");
    }
    bool indicator = true;
    size_t delete_index = 0;
    for (; delete_index < current_size; delete_index++) {
        if (strncmp(pairs[delete_index].key, key, max_key_length - 1) == 0) {
            break;
        }
    }
    if (delete_index == current_size) {
        return;
    }
    Pair* new_pairs = new Pair[current_size - 1];
    memcpy(new_pairs, pairs, sizeof(Pair) * delete_index);
    if (delete_index + 1 < current_size) {
        memcpy(new_pairs + delete_index, pairs + delete_index + 1, sizeof(Pair) * (current_size - delete_index - 1));
    }
    delete[] pairs;
    pairs = new_pairs;
    current_size -= 1;
}

const int& Map::operator[](const char* key) const {
    if (key == nullptr || key[0] == 0) {
        throw std::invalid_argument("invalid key");
    }
    if (const_cast<Map*>(this)->find(key) == nullptr) {
        throw std::out_of_range("out of range");
    }
    return const_cast<Map*>(this)->find(key)->value;
}

int& Map::operator[](const char* key) {
    if (key == nullptr || key[0] == 0) {
        throw std::invalid_argument("invalid key");
    }
    return insert(key);
}
