#include <sstream>

#include "utilities.hpp"

namespace {
    std::string StringReplacer(const std::string& inputStr, const int size, const char* oldS, const char* newS) {
        std::string result(inputStr);
        size_t pos = result.find(oldS);
        while (pos != std::string::npos) {
            result.replace(pos, size, newS);
            pos = result.find(oldS, pos);
        }
        return result;
    }

    void literallReplace(std::string& s) {
        s = StringReplacer(s, 2, "\\n", "\n");
        s = StringReplacer(s, 2, "\\\"", "\"");
        s = StringReplacer(s, 2, "\\\\", "\\");
    }
}

bool inQuotes(const std::string& str) {
    return str.front() == '\"' && str.back() == '\"' && str.size() > 1;
}

void deleteQuotes(std::string& str) {
    if (!inQuotes(str))
        return;
    str.erase(0, 1);
    str.pop_back();
}

int convertToInt(const std::string& numStr) {
    std::stringstream ss(numStr);
    int num;
    ss >> num;
    return num;
}

std::vector<std::string> createWordList(const std::string& str) {
    std::vector<std::string> wordList;
    std::stringstream input(str);
    std::string word, subWord;
    while (input >> word) {
        if (word[0] == '\"') {
            if (word == "\"")
                word += input.get();
            auto len = word.length();
            char prev = word[len - 2];
            if (!(len > 3 && word[len - 3] == '\\' && word[len - 2] == '\\' && word[len - 1] == '\"')) {
                while (word.back() != '\"' && word.back() != EOF || prev == '\\' && word.back() == '\"') {
                    if (prev == '\\' && word.back() == '\\' && input.peek() == '\"') {
                        word += input.get();
                        break;
                    }
                    prev = word.back();
                    word += input.get();
                }
            }
        }
        literallReplace(word);
        wordList.push_back(word);
    }
    return wordList;
}
