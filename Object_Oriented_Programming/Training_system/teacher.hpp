#pragma once

#include <stdexcept>
#include <iostream>

#include "course.hpp"

class Teacher : public Item {
private:
    size_t experience{};
    std::vector<std::weak_ptr<Course>> courses{};

public:
    Teacher(const std::string& name, size_t id, size_t experience_, const vector_ids& ids)
        : Item(name, id, ids), experience(experience_) {}

    size_t getExperience() { return experience; }

    void addItem(std::weak_ptr<Item> item) override {
        auto ptr = item.lock();
        if (!ptr) {
            throw std::invalid_argument("Course is already deleted!");
        }
        auto concrete_ptr = std::dynamic_pointer_cast<Course>(ptr);
        if (concrete_ptr == nullptr) {
            throw std::invalid_argument("Item isn't Course");
        }
        if (std::find(getIDs().begin(), getIDs().end(), concrete_ptr->getID()) == getIDs().end()) {
            return;
        }
        if (std::find_if(getCourses().begin(), getCourses().end(), [&concrete_ptr](std::weak_ptr<Course> course) {
                return course.lock()->getID() == concrete_ptr->getID();
            }) == getCourses().end()) {
            courses.push_back(concrete_ptr);
        }
    }

    const std::vector<std::weak_ptr<Course>>& getCourses() const { return courses; }

    void serialize(std::ostream& os) override {
        os << "teacher\t" << getName() << "\t" << getID() << "\t" << getExperience();
        for (auto& it : courses) {
            os << "\t" << it.lock()->getID();
        }
        os << std::endl;
    }
};
