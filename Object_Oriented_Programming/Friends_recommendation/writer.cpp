#include "writer.hpp"

void Writer::writeRecommendedFriends(const std::vector<std::string>& friends) {
    if (friends.empty()) return;
    os << "Recommendation for " << friends[0] << '\n';
    for (size_t i = 1; i < friends.size(); i++) {
        os << friends[i] << '\n';
    }
}
