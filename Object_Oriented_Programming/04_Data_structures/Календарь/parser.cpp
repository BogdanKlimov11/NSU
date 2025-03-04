#include <map>
#include <set>
#include <algorithm>

#include "parser.hpp"
#include "number_of_days.hpp"

using namespace std;

namespace {
    const map<string, RangeType> Ranges = {{"range", RangeType::RANGE},
                                           {"year", RangeType::YEAR},
                                           {"month", RangeType::MONTH}};
    const map<string, bool> Orientations = {{"vert", true},
                                            {"horiz", false}};
    const set<string> YearTypes = {"year_once", "year_for_every_month"};

    bool IsNumber(std::string word) {
        try {
            size_t length;
            stoi(word, &length);
            return length == word.length();
        } catch (exception& exception) {
            return false;
        }
    }
}

Range CreateRange(std::vector<std::string> source) {
    Range range{};
    vector<int> period;
    for (auto line : source) {
        for (size_t notSpace = 0;;) {
            notSpace = line.find_first_not_of(" \t", notSpace);
            if (notSpace == std::string::npos)
                break;
            size_t isSpace = line.find_first_of(" \t", notSpace);
            size_t length = (isSpace == std::string::npos) ? std::string::npos : isSpace - notSpace;
            std::string word(line.substr(notSpace, length));

            map<string, RangeType>::iterator rangePosition;
            map<string, bool>::iterator orientPosition;
            set<string>::iterator yearTypePosition;
            if ((rangePosition = Ranges.find(word)) != Ranges.end()) {
                range.type = rangePosition->second;
            } else if (IsNumber(word)) {
                period.push_back(stoi(word));
            } else if ((orientPosition = Orientations.find(word)) != Orientations.end()) {
                range.orient = orientPosition->second;
            } else if ((yearTypePosition = YearTypes.find(word)) != YearTypes.end()) {
                if (word == "year_for_every_month")
                    range.yearEveryMonth = true;
                else if (word == "year_once")
                    range.yearOnce = true;
            } else if (word == "|") {
                notSpace = isSpace;
                continue;
            } else {
                range.type = RangeType::UNDEFINED;
            }
            if (isSpace == std::string::npos)
                break;
            notSpace = isSpace;
        }
        switch (range.type) {
            case RangeType::RANGE:
                if (period.size() != 4) {
                    range.type = RangeType::UNDEFINED;
                    return range;
                }
                range.dateBegin = Date(1, period[0], period[1]);
                range.dateEnd = Date(numberOfDays(period[2] - 1, period[3]), period[2], period[3]);
                break;
            case RangeType::YEAR:
                if (period.size() != 2) {
                    range.type = RangeType::UNDEFINED;
                    return range;
                }
                range.dateBegin = Date(1, 1, period[0]);
                range.dateEnd = Date(31, 12, period[0]);
                break;
            case RangeType::MONTH:
                if (period.size() != 2) {
                    range.type = RangeType::UNDEFINED;
                    return range;
                }
                range.dateBegin = Date(1, period[0], period[1]);
                range.dateEnd = Date(numberOfDays(period[0] - 1, period[1]), period[0], period[1]);
                break;
            default:
                break;
        }
        if (range.dateBegin == Date(-1, -1, -1) || range.dateEnd == Date(-1, -1, -1)) {
            range.type = RangeType::UNDEFINED;
        }
        return range;
    }
    range.type = RangeType::UNDEFINED;
    return range;
}

vector<string> streamReader(istream& input) {
    vector<string> read;
    string line;
    while (getline(input, line))
        read.push_back(line);
    return read;
}
