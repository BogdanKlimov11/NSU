#include "parser.hpp" 

std::pair<Point, bool> Parser::getNextPoint() const {
    std::string line;
    size_t ver_id;
    int coord1, coord2;
    char separator;
    while (getline(is, line)) {
        std::istringstream iss(line);
        if (!(iss >> ver_id) || !(iss >> separator)) continue;
        if (!(iss >> coord1) || !(iss >> separator)) continue;
        if (!(iss >> coord2)) continue;
        
        return std::make_pair(Point(ver_id, coord1, coord2), true);
    }
    
    return std::make_pair(Point(), false);
}
