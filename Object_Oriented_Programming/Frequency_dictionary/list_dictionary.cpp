#include "list_dictionary.hpp"

void ListDictionary::addWord(const std::string& word) {
    if (!word.empty()) {
        auto it = findWord(word);
        if (it == Dictionary.end()) {
            Dictionary.push_back(std::make_pair(word, 1));
        }
        else {
            it->second++;
        }
    }
}

bool ListDictionary::ResetEnumerator() {
    if (!Dictionary.empty()) {
        DictIterator = Dictionary.begin();
        return true;
    }
    return false;
}

std::pair<std::string, size_t> ListDictionary::getNext() {
    if (Dictionary.empty())
        throw std::out_of_range("Dictionary is empty");

    if (DictIterator != Dictionary.end()) {
        std::pair<std::string, size_t> current = *DictIterator;
        DictIterator++;
        return current;
    }
    return std::make_pair(std::string(), 0);
}

std::list<std::pair<std::string, size_t>>::iterator ListDictionary::findWord(const std::string& word) {
    if (word.empty()) {
        throw std::invalid_argument("Empty string");
    }

    auto it = Dictionary.begin();
    for (; it != Dictionary.end(); it++) {
        if (it->first == word)
            break;
    }
    return it;
}
