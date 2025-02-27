#include "code_review_decorator.hpp"

const char* CodeReviewDecorator::forbidden = "#&0123456789-.,:;+=()[]{}!?<>\"\' ";

std::string CodeReviewDecorator::write(const std::string& str) {
    auto words = BuildColorMap(str, key_words);
    return Decorate(Colorize(str, words));
}

std::map<std::string, Color> CodeReviewDecorator::BuildColorMap(const std::string& str, const std::map<std::string, Color>& special_words) {
    if (str.empty())
        return std::map<std::string, Color>();

    std::map<std::string, Color> color_map;
    std::stringstream str_stream(str);
    std::string word;

    while (str_stream >> word) {
        auto filtered_words = filterPunctuatuion(word);
        for (auto& fword : filtered_words) {
            auto it = special_words.find(fword);
            if (it == special_words.end()) {
                auto punct = fword.find_first_of(forbidden, 0);
                if ((punct == 0 && fword.length() == 1) || (special_words.find(fword) != special_words.end())) {
                    continue;
                }
                it = color_map.insert({fword, Color::generate()}).first;
            }
        }
    }
    return color_map;
}

std::vector<std::string> CodeReviewDecorator::filterPunctuatuion(const std::string& str) {
    std::vector<std::string> result;
    std::string word;
    size_t it = 0;

    while (it < str.length()) {
        auto punct = str.find_first_of(forbidden, it);
        if (punct == std::string::npos) {
            result.push_back(str.substr(it, str.length()));
            break;
        }
        else if (punct != it) {
            word = str.substr(it, punct - it);
            result.push_back(word);
        }
        word = str.substr(punct, 1);
        result.push_back(word);
        it = punct + 1;
    }
    return result;
}
