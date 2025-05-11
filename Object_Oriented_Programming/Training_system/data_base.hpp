#pragma once

#include <unordered_map>

#include "data_base_filler.hpp"

class DataBase final {
private:
    std::unordered_map<size_t, std::shared_ptr<Item>> items;
    bool is_acutal = false;
    std::vector<std::shared_ptr<Course>> Courses;
    std::vector<std::shared_ptr<Teacher>> Teachers;
    std::vector<std::shared_ptr<Student>> Students;

public:
    DataBase() = default;
    void fill(DataBaseFiller& db_filler);
    void addItem(std::shared_ptr<Item> item);
    std::shared_ptr<Student> getStudent(size_t id);
    std::shared_ptr<Teacher> getTeacher(size_t id);
    std::shared_ptr<Course> getCourse(size_t id);
    std::shared_ptr<Item> getItem(size_t id);
    void serialize(std::ostream& os);
    void update();
};
