#pragma once

#include <unordered_map>

#include "strategy.hpp"

template<class Key>
class CountStrategy final : public Strategy<Key> {
    using UMIterator = typename std::unordered_map<Key, size_t>::iterator;

    size_t lifeTime;
    std::unordered_map<Key, size_t> KeysCounter;

public:
    CountStrategy(size_t _lifeTime) : lifeTime(_lifeTime) {}

    bool insert(const Key& key) override {
        std::pair<UMIterator, bool> tmp = KeysCounter.insert(std::pair<const Key, size_t>(key, 0));
        return tmp.second;
    }

    void erase(const Key& key) override {
        KeysCounter.erase(key);
    }

    bool check(const Key& key) override {
        return (KeysCounter.find(key) != KeysCounter.end()) && (KeysCounter[key] != lifeTime);
    }

    bool access(const Key& key) override {
        if (KeysCounter.find(key) != KeysCounter.end() && KeysCounter[key] != lifeTime) {
            ++KeysCounter[key];
            return true;
        }
        return false;
    }
};
