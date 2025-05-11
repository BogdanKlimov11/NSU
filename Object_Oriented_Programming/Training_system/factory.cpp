#include "factory.hpp"

std::shared_ptr<Item> Factory::create(const std::string& type, const std::string& name, size_t id, size_t info, std::vector<size_t> items) {
    auto it = creators.find(type);
    if (it == creators.end()) {
        throw std::invalid_argument("No such type of item");
    }
    auto& creator_ptr = it->second;
    auto item = creator_ptr->create(name, id, info, items);
    return item;
}
