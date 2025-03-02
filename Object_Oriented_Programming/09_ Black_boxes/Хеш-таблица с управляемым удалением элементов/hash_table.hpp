#pragma once

#include <functional>
#include <unordered_map>
#include <vector>
#include <map>
#include <initializer_list>

#include "strategy.hpp"

template <class Key, class Value>
class HashTable {
private:
    Strategy<Key>& strategy;
    std::unordered_map<Key, Value> UMap;

    friend class HashTableIterator;
    class HashTableIterator {
        friend HashTable;
        using Iterator = typename std::unordered_map<Key, Value>::iterator;

    private:
        HashTable& parent;
        Iterator iter;

    public:
        using difference_type = typename HashTable::size_type;
        using value_type = typename HashTable::value_type;
        using mapped_type = typename HashTable::mapped_type;
        using key_type = typename HashTable::key_type;
        using reference = typename HashTable::reference;
        using iterator_category = std::forward_iterator_tag;

        HashTableIterator(HashTable& _parent, Iterator _it) : parent{_parent}, iter{_it} {}

        HashTableIterator& operator++() {
            ++iter;
            while (iter != parent.UMap.end()) {
                Key key = (*iter).first;
                if (parent.strategy.check(key)) {
                    break;
                }
                ++iter;
            }
            return *this;
        }

        reference operator*() {
            Key key = (*iter).first;
            bool access = parent.strategy.access(key);
            if (!access) {
                throw std::out_of_range("Invalid key");
            }
            return *iter;
        }

        bool operator!=(const HashTableIterator& other) const { return iter != other.iter; }
    };

    HashTableIterator getIterator(const Key& key) {
        return HashTableIterator(*this, UMap.find(key));
    }

public:
    using umap_type = std::unordered_map<Key, Value>;
    using UMapIterator = typename std::unordered_map<Key, Value>::iterator;
    using value_type = std::pair<const Key, Value>;
    using size_type = size_t;
    using key_type = Key;
    using mapped_type = Value;
    using reference = value_type&;

    HashTable(Strategy<Key>& _strat) : strategy(_strat) {}
    ~HashTable() = default;

    std::pair<HashTableIterator, bool> insert(const Key& key, const Value& val) {
        if (UMap.find(key) != UMap.end())
            throw std::invalid_argument("Such Key is already exist!");
        bool success = strategy.insert(key);
        if (success) {
            std::pair<UMapIterator, bool> tmp_pair = UMap.insert(std::make_pair(key, val));
            HashTableIterator tmp(*this, tmp_pair.first);
            return std::pair<HashTableIterator, bool>(tmp, tmp_pair.second);
        }
        return std::pair<HashTableIterator, bool>(getIterator(key), false);
    }

    size_type erase(const Key& key) {
        if (UMap.find(key) == UMap.end())
            throw std::invalid_argument("Invalid key");
        strategy.erase(key);
        return UMap.erase(key);
    }

    mapped_type& operator[](const Key& key) {
        if (!strategy.access(key)) {
            if (UMap.find(key) != UMap.end()) {
                strategy.erase(key);
                erase(key);
            }
            bool success = strategy.insert(key);
            if (!success) {
                throw std::out_of_range("Can't insert the key");
            }
        }
        return UMap[key];
    }

    mapped_type& at(const Key& key) {
        if (UMap.find(key) == UMap.end())
            throw std::invalid_argument("Invalid key");
        if (!strategy.access(key)) {
            erase(key);
            throw std::out_of_range("key is expired");
        }
        return UMap.at(key);
    }

    HashTableIterator begin() {
        HashTableIterator tmp(*this, UMap.begin());
        return tmp;
    }

    HashTableIterator end() {
        HashTableIterator tmp(*this, UMap.end());
        return tmp;
    }
};
