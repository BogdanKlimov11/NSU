#include "friend_recommendation.h"

void FriendRecommendation::friendFiller(Parser & parser) {
    while (true) {
        auto friendship = parser.getNextFriendship();
        if (!friendship.second) break;

        AddFriendship(friendship.first.first, friendship.first.second);
    }
}

std::vector<std::string> FriendRecommendation::RecommendTo(const std::string& uid, size_t percent) {
    if (uid.empty() || percent > 100) {
        throw std::invalid_argument("Incorrect input: uid is empty or percent > 100");
    }
    std::unordered_map<std::string, size_t> allFriendsOfFriends;
    const auto& friends = UserDataBase.at(uid);
    for (auto &it : friends) {
        const auto& friends_of_friend = UserDataBase.at(it);
        for (auto &it_fof : friends_of_friend) {
            if ((it_fof != uid) && (UserDataBase.at(uid).count(it_fof) == 0)) {
                allFriendsOfFriends[it_fof]++;
            }
        }
    }
    std::vector<std::string> result;
    result.push_back(uid);
    for (auto &it : allFriendsOfFriends) {
        if (it.second * 100 / friends.size() >= percent) {
            result.push_back(it.first);
        }
    }
    return result;
}

void FriendRecommendation::AddFriendship(const std::string& uid1, const std::string& uid2) {
    if (uid1.empty() || uid2.empty()) {
        throw std::invalid_argument("User is incorrect!");
    }
    if (uid1 == uid2) {
        return;
    }
    UserDataBase[uid1].insert(uid2);
    UserDataBase[uid2].insert(uid1);
}

const std::unordered_set<std::string>& FriendRecommendation::getFriendsOf(const std::string & uid) {
    if (uid.empty()) {
        throw std::invalid_argument("Incorrect user id!");
    }
    return UserDataBase.at(uid);
}
