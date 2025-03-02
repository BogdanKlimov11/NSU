#pragma once

#include <map>

#include "creator.hpp"
#include "teacher.hpp"
#include "student.hpp"
#include "course.hpp"

class Factory {
private:
    std::map<std::string, std::shared_ptr<AbstractCreator>> creators = {
        {"teacher", std::shared_ptr<AbstractCreator>(new Creator<Teacher>())},
        {"course", std::shared_ptr<AbstractCreator>(new Creator<Course>())},
        {"student", std::shared_ptr<AbstractCreator>(new Creator<Student>())}
    };

public:
    Factory() = default;
    std::shared_ptr<Item> create(const std::string& type, const std::string& name, size_t id, size_t info, std::vector<size_t> items);
};
