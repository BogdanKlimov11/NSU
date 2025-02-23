#pragma once

#include "graph.hpp"

class Writer {
private:
    std::ostream& outputStream;

public:
    Writer(std::ostream& os) : outputStream(os) {}
    void writeMSTVector(const std::vector<std::pair<double, edge>>& mst);
};
