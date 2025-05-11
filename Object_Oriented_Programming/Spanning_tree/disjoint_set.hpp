#pragma once

#include <vector>
#include <iostream>
#include <algorithm>

class DisjointSet final {
private:
    std::vector<size_t> parent;
    std::vector<size_t> rank;
    size_t size;

public:
    DisjointSet(size_t n);
    ~DisjointSet() = default;
    size_t find(size_t u);
    void setUnion(size_t v1, size_t v2);
};
