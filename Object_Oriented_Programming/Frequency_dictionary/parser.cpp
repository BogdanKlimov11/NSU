#include "parser.hpp"

std::pair<std::vector<std::string>, bool> Parser::getWord() {
    std::string word;
    std::vector<std::string> filtered;
    if (is_ >> word) {
        filtered = filterPunctuatuion(word);
        if (!filtered.empty()) {
            return std::make_pair(filtered, true);
        }
    }
    return std::make_pair(filtered, false);
}

std::vector<std::string> Parser::filterPunctuatuion(const std::string& str) {
    std::vector<std::string> result;
    const char* forbidden{"—.,:;+=()[]{}!? "};
    size_t it = 0;

    while (it < str.length()) {
        auto punct = str.find_first_of(forbidden, it);
        if (punct == std::string::npos) {
            result.push_back(str.substr(it, str.length()));
            break;
        }
        else if (punct != it) {
            result.push_back(str.substr(it, punct - it));
        }
        it = punct + 1;
    }
    return result;
}

void Parser::fillDictionary(FrequencyDictionary* dict) {
    if (dict == nullptr)
        throw std::invalid_argument("Empty dictionary!");

    while (true) {
        std::pair<std::vector<std::string>, bool> words = getWord();
        if (!words.second)
            break;
        for (auto it = words.first.begin(); it < words.first.end(); it++) {
            dict->addWord(*it);
        }
    }
}
