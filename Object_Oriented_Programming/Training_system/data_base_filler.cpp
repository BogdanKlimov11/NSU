#include "data_base_filler.hpp"

std::shared_ptr<Item> DataBaseFiller::getNextItem() {
    std::string line{};
    while (std::getline(is, line)) {
        std::istringstream iss(line);
        std::string type{};
        std::string name{};
        size_t id{}, info{}, tmp{};
        std::vector<size_t> ids;
      
        if (!(iss >> type)) continue;
        if (!(iss >> name)) continue;
        if (!(iss >> id)) continue;
        if (!(iss >> info)) continue;
        while ((iss >> tmp)) {
            ids.push_back(tmp);
        }
        try {
            auto new_item = factory.create(type, name, id, info, ids);
            return new_item;
        }
        catch (std::invalid_argument) { continue; }
    }
    
    return nullptr;
}
