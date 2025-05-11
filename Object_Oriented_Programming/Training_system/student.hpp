#pragma once

#include <iostream>
#include <stdexcept>

#include "item.hpp"
#include "course.hpp"

class Course;

class Student : public Item {
private:
    std::vector<std::weak_ptr<Course>> courses{};

public:
    Student(const std::string& name, size_t id, size_t course, const vector_ids& ids);
    virtual void addItem(std::weak_ptr<Item> item) override;
    const std::vector<std::weak_ptr<Course>>& getCourses() const;
    virtual void serialize(std::ostream& os) override;
};
