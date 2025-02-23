#include "writer.hpp"

void Writer::writeMSTVector(const std::vector<std::pair<double, edge>> & mst) {
    if (mst.empty()) return;
    size_t size = mst.size();
    for (size_t i = 0; i < size; i++) {
        outputStream << mst[i].second.first << " - "
            << mst[i].second.second << " : "
            << mst[i].first << '\n';
    }
}
