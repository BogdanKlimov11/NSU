#pragma once

#include <unordered_map>
#include <unordered_set>

#include "parser.h"

class FriendRecommendation final {
private:
    std::unordered_map<std::string, std::unordered_set<std::string>> UserDataBase;

public:
    FriendRecommendation() = default;
    ~FriendRecommendation() = default;

    void friendFiller(Parser & parser);
    std::vector<std::string> RecommendTo(const std::string& uid, size_t percent);
    void AddFriendship(const std::string& uid1, const std::string& uid2);
    const std::unordered_set<std::string>& getFriendsOf(const std::string & uid);
};
