#pragma once

#include <iostream>
#include <stdexcept>

#include "student.hpp"

class Student;

class Course : public Item {
private:
    size_t teacher_id{};
    std::vector<std::weak_ptr<Student>> students{};

public:
    Course(const std::string& name, size_t id, size_t teacher_id_, const vector_ids& ids) :
        Item(name, id, ids), teacher_id(teacher_id_) {}
    size_t getTeacherID() const;
    virtual void addItem(std::weak_ptr<Item> item) override;
    const std::vector<std::weak_ptr<Student>>& getStudents() const;
    virtual void serialize(std::ostream& os) override;
};
