#include "data_base.hpp"

void DataBase::fill(DataBaseFiller& db_filler) {
    while (true) {
        auto new_item = db_filler.getNextItem();
        if (!new_item)
            break;
        addItem(new_item);
    }
}

void DataBase::addItem(std::shared_ptr<Item> item) {
    if (!is_acutal)
        items.insert({item->getID(), item});
    else
        throw std::out_of_range("Data Base is already updated");
}

std::shared_ptr<Student> DataBase::getStudent(size_t id) {
    auto item = getItem(id);
    auto student = std::dynamic_pointer_cast<Student>(item);
    return student;
}

std::shared_ptr<Teacher> DataBase::getTeacher(size_t id) {
    auto item = getItem(id);
    auto teacher = std::dynamic_pointer_cast<Teacher>(item);
    return teacher;
}

std::shared_ptr<Course> DataBase::getCourse(size_t id) {
    auto item = getItem(id);
    auto course = std::dynamic_pointer_cast<Course>(item);
    return course;
}

std::shared_ptr<Item> DataBase::getItem(size_t id) {
    if (!is_acutal) {
        update();
    }
    auto& item = items.at(id);
    return item;
}

void DataBase::serialize(std::ostream& os) {
    if (items.empty())
        return;
    if (!is_acutal) {
        update();
    }
    for (auto& course : Courses) {
        course->serialize(os);
    }
    for (auto& teacher : Teachers) {
        teacher->serialize(os);
    }
    for (auto& student : Students) {
        student->serialize(os);
    }
}

void DataBase::update() {
    if (items.empty()) {
        throw std::invalid_argument("Incorrect input: data base is empty");
    }
    if (is_acutal) {
        return;
    }
    for (auto& item : items) {
        auto& current_item = item.second;
        auto indexes_of_item = current_item->getIDs();
        for (auto id : indexes_of_item) {
            auto it = items.find(id);
            if (it == items.end()) {
                continue;
            }
            if (!it->second) {
                items.erase(it->first);
                continue;
            }
            try {
                auto& sub_item = it->second;
                current_item->addItem(sub_item);
                sub_item->addItem(current_item);
            } catch (const std::invalid_argument&) {
                continue;
            }
        }
        {
            auto concete_item = std::dynamic_pointer_cast<Course>(current_item);
            if (concete_item) {
                Courses.push_back(concete_item);
                continue;
            }
        }
        {
            auto concete_item = std::dynamic_pointer_cast<Teacher>(current_item);
            if (concete_item) {
                Teachers.push_back(concete_item);
                continue;
            }
        }
        {
            auto concete_item = std::dynamic_pointer_cast<Student>(current_item);
            if (concete_item) {
                Students.push_back(concete_item);
                continue;
            }
        }
    }
    is_acutal = true;
}
