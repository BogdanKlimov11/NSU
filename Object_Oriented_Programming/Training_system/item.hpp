#pragma once

#include <vector>
#include <memory>
#include <string>

class Item {
protected:
    using vector_sh_ptrs = std::vector<std::shared_ptr<Item>>;
    using vector_w_ptrs = std::vector<std::weak_ptr<Item>>;
    using vector_ids = std::vector<size_t>;
    
    std::string name{};
    size_t id{};
    vector_ids item_ids{};

public:
    Item() = default;
    Item(const std::string& name_, size_t id_, const vector_ids& ids_) : name(name_), id(id_), item_ids(ids_) {};
    virtual size_t getID() { return id; }
    const std::string& getName() const { return name; }
    const vector_ids& getIDs() const { return item_ids; }
    virtual void addItemID(size_t new_id) { item_ids.push_back(new_id); }
    virtual void addItem(std::weak_ptr<Item> item) = 0;
    virtual void serialize(std::ostream& os) = 0;
    virtual ~Item() = default;
};
