#include "real_spending_parser.hpp"

std::pair<Spending, bool> RealSpendingParser::getNextItem() {
    std::string line, str_path;
    double value;
    std::string strdate;
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        if (!(iss >> strdate))
            continue;
        if (!(iss >> str_path))
            continue;
        if (!(iss >> value))
            continue;
        std::pair<std::string, std::string> path = getPath(str_path);
        Date date{strdate};
        return std::make_pair(Spending(date, path.first, path.second, value), true);
    }
    return std::make_pair(Spending(), false);
}

std::pair<std::string, std::string> RealSpendingParser::getPath(const std::string& str) {
    if (str.empty()) {
        throw std::invalid_argument("Can't get path: empty string");
    }
    std::pair<std::string, std::string> path;
    size_t idx = str.find(':');
    if (idx != std::string::npos) {
        path = std::make_pair(str.substr(0, idx), str.substr(idx + 1, str.length()));
    } else {
        path = std::make_pair(str.substr(0, str.length()), "");
    }
    return path;
}
