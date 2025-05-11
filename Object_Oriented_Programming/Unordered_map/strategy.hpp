#pragma once

template<class Key>
class Strategy {
public:
    virtual ~Strategy() = default;
    virtual bool insert(const Key& key) = 0;
    virtual bool check(const Key& key) = 0;
    virtual bool access(const Key& key) = 0;
    virtual void erase(const Key& key) = 0;
};
