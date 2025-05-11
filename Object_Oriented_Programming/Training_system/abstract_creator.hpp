#pragma once

#include "item.hpp"

class AbstractCreator {
public:
    virtual std::shared_ptr<Item> create(const std::string& name, size_t id, size_t info, std::vector<size_t>& items) = 0;
};
