#pragma once

#include <iostream>
#include <vector>
#include <sstream>
#include <string>

class Writer {
private:
    std::ostream& os;

public:
    Writer(std::ostream& _os) : os(_os) {}
    void writeRecommendedFriends(const std::vector<std::string>& friends);
};
