#include <string>
#include <vector>
#include <sstream>
#include <utility>

#include "expected_spending_parser.hpp"

std::pair<Categories, bool> ExpectedSpendingParser::getNextItem() {
    std::string line, str_path;
    double value;
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        if (!(iss >> str_path))
            continue;
        if (!(iss >> value))
            continue;
        std::vector<CategoryItem> paths = getPath(str_path);
        return std::make_pair(Categories{paths, value}, true);
    }
    return std::make_pair(Categories{}, false);
}

std::vector<CategoryItem> ExpectedSpendingParser::getPath(const std::string& str) {
    if (str.empty()) {
        throw std::invalid_argument("Can't get path: empty string");
    }

    std::vector<CategoryItem> paths;
    std::vector<std::string> path;
    const char* forbidden{":+"};
    size_t it = 0;
    while (it < str.length()) {
        const size_t punct(str.find_first_of(forbidden, it));
        if (punct == std::string::npos) {
            path.push_back(str.substr(it, str.length()));
            if (path.size() == 2) {
                CategoryItem item{path[0], path[1]};
                paths.push_back(item);
            } else {
                CategoryItem item{path[0], ""};
                paths.push_back(item);
            }
            break;
        } else if ((punct != it) && (str.at(punct) == ':')) {
            path.push_back(str.substr(it, punct - it));
        } else if ((punct != it) && (str.at(punct) == '+')) {
            path.push_back(str.substr(it, punct - it));
            if (path.size() == 2) {
                CategoryItem item{path[0], path[1]};
                paths.push_back(item);
            } else {
                CategoryItem item{path[0], ""};
                paths.push_back(item);
            }
            path.clear();
        }
        it = punct + 1;
    }
    return paths;
}
