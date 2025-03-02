#pragma once

#include "abstract_creator.hpp"

template <class Object>
class Creator : public AbstractCreator {
public:
    Creator() = default;
    virtual std::shared_ptr<Item> create(const std::string& name, size_t id, size_t info, std::vector<size_t>& items_id) override {
        return std::shared_ptr<Item>(new Object(name, id, info, items_id));
    }
};
