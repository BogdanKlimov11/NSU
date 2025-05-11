#include "student.hpp"

Student::Student(const std::string& name, size_t id, size_t course, const vector_ids& ids) : 
    Item(name, id, ids) {
    item_ids.insert(item_ids.begin(), course);
}

void Student::addItem(std::weak_ptr<Item> item) {
    auto ptr = item.lock();
    if (!ptr) {
        throw std::invalid_argument("Course is already deleted!");
    }
    auto concrete_ptr = std::dynamic_pointer_cast<Course>(ptr);
    if (concrete_ptr == nullptr) {
        throw std::invalid_argument("item isn't Course");
    }
    if (std::find(getIDs().begin(), getIDs().end(), concrete_ptr->getID()) == getIDs().end()) {
        return;
    }
    if (std::find_if(getCourses().begin(), getCourses().end(), [&concrete_ptr](std::weak_ptr<Course> course) { return course.lock()->getID() == concrete_ptr->getID(); }) == getCourses().end()) {
        courses.push_back(concrete_ptr);
    }
}

const std::vector<std::weak_ptr<Course>>& Student::getCourses() const {
    return courses;
}

void Student::serialize(std::ostream& os) {
    os << "student\t" << getName() << "\t" << getID();
    for (auto& it : courses) {
        os << "\t" << it.lock()->getID();
    }
    os << std::endl;
}
