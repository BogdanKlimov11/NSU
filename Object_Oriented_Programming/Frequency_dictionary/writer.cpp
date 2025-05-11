#include "writer.hpp"

void writeDictionary(std::ostream& os, FrequencyDictionary* dict) {
    if (dict == nullptr) {
        throw std::invalid_argument("Empty dictionary!");
    }

    bool dict_isnot_empty = dict->ResetEnumerator();
    if (dict_isnot_empty) {
        auto isend = dict->IteratorIsEnd();
        while (!isend) {
            std::pair<std::string, size_t> word = dict->getNext();
            os << word.first << " : " << word.second << '\n';
            isend = dict->IteratorIsEnd();
        }
    }
}
