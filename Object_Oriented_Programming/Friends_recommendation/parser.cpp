#include "parser.hpp"

std::pair<std::pair<std::string, std::string>, bool> Parser::getNextFriendship() {
    std::string line, uid1, uid2;
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        if (!(iss >> uid1)) continue;
        if (!(iss >> uid2)) continue;

        return std::make_pair(std::make_pair(uid1, uid2), true);
    }
    return std::make_pair(std::make_pair(std::string(), std::string()), false);
}
