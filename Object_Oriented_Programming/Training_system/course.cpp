#pragma once

#include "course.hpp"

size_t Course::getTeacherID() const {
    return teacher_id;
}

void Course::addItem(std::weak_ptr<Item> item) {
    auto ptr = item.lock();
    if (!ptr) {
        throw std::invalid_argument("Course is already deleted!");
    }
    auto concrete_ptr = std::dynamic_pointer_cast<Student>(ptr);
    if (concrete_ptr == nullptr) {
        throw std::invalid_argument("Item isn't Student!");
    }
    if (std::find(getIDs().begin(), getIDs().end(), concrete_ptr->getID()) == getIDs().end()) {
        return;
    }
    if (std::find_if(getStudents().begin(), getStudents().end(), [&concrete_ptr](std::weak_ptr<Student> student) { return student.lock()->getID() == concrete_ptr->getID(); }) == getStudents().end()) {
        students.push_back(concrete_ptr);
    }
}

const std::vector<std::weak_ptr<Student>>& Course::getStudents() const {
    return students;
}

void Course::serialize(std::ostream& os) {
    os << "course\t" << getName() << "\t" << getID() << "\t" << getTeacherID();
    for (auto& it : students) {
        os << "\t" << it.lock()->getID();
    }
    os << std::endl;
}
