#include <gtest/gtest.h>

#include "data_base.hpp"

using vector_ids = std::vector<size_t>;
using vector_sh_ptrs = std::vector<std::shared_ptr<Item>>;

TEST(Student, OneElem) {
    Student s("Alex", 1, 1, vector_ids());
    EXPECT_EQ(s.getName(), "Alex");
    EXPECT_EQ(s.getID(), 1);
    EXPECT_EQ(s.getIDs(), vector_ids{1});
}

TEST(Teacher, EmptyVec) {
    Teacher t("Alexander", 1, 2, vector_ids());
    EXPECT_EQ(t.getName(), "Alexander");
    EXPECT_EQ(t.getID(), 1);
    EXPECT_EQ(t.getExperience(), 2);
    EXPECT_EQ(t.getIDs(), vector_ids());
}

TEST(Course, EmptyVec) {
    Course c("Physics", 4, 1, vector_ids());
    EXPECT_EQ(c.getName(), "Physics");
    EXPECT_EQ(c.getID(), 4);
    EXPECT_EQ(c.getTeacherID(), 1);
    EXPECT_EQ(c.getIDs(), vector_ids());
}

TEST(TeacherCreator, EmptyIds) {
    auto c = new Creator<Teacher>();
    auto t1 = c->create("Alexander", 1, 2, vector_ids());
    Teacher t("Alexander", 1, 2, vector_ids());
    EXPECT_EQ(t.getName(), t1->getName());
    EXPECT_EQ(t.getID(), t1->getID());
    auto d = std::dynamic_pointer_cast<Teacher>(t1);
    EXPECT_EQ(t.getExperience(), d->getExperience());
    EXPECT_EQ(t.getIDs(), d->getIDs());
}

TEST(CourseCreator, FewStudents) {
    vector_sh_ptrs students = {
        std::shared_ptr<Item>(new Student("a", 1, 1, vector_ids{2, 3, 4})),
        std::shared_ptr<Item>(new Student("b", 2, 1, vector_ids{1, 3, 4})),
        std::shared_ptr<Item>(new Student("c", 3, 1, vector_ids{1, 2, 4}))
    };
    auto c = new Creator<Course>();
    auto t1 = c->create("physics", 7, 78, vector_ids{1, 2, 3});
    Course t("physics", 7, 78, vector_ids{1, 2, 3});
    EXPECT_EQ(t.getName(), t1->getName());
    EXPECT_EQ(t.getID(), t1->getID());
    auto d = std::dynamic_pointer_cast<Course>(t1);
    EXPECT_EQ(t.getTeacherID(), d->getTeacherID());
    EXPECT_EQ(t.getIDs(), d->getIDs());
}

TEST(Students, Downcast) {
    vector_ids courses{1, 2, 3};
    auto creator = new Creator<Student>();
    auto student1 = creator->create("alex", 7, 78, courses);
    Student student("alex", 7, 78, courses);
    EXPECT_EQ(student.getName(), student1->getName());
    EXPECT_EQ(student.getID(), student1->getID());
    auto d = std::dynamic_pointer_cast<Student>(student1);
    EXPECT_EQ(student.getName(), d->getName());
    EXPECT_EQ(student.getIDs(), d->getIDs());
    EXPECT_EQ(student1->getID(), student1->getID());
}

TEST(Items, AddIds) {
    auto a = std::shared_ptr<Course>(new Course("a", 1, 2, vector_ids{11}));
    auto b = std::shared_ptr<Teacher>(new Teacher("b", 2, 1, vector_ids{12}));
    auto c = std::shared_ptr<Student>(new Student("c", 3, 1, vector_ids{13}));
    a->addItemID(c->getID());
    b->addItemID(a->getID());
    c->addItemID(a->getID());
    vector_ids s_courses{1, 13, 1};
    vector_ids t_courses{12, 1};
    vector_ids students{11, 3};
    EXPECT_EQ(c->getIDs(), s_courses);
    EXPECT_EQ(b->getIDs(), t_courses);
    EXPECT_EQ(a->getIDs(), students);
}

TEST(Creators, Upcast) {
    std::shared_ptr<AbstractCreator> a(new Creator<Teacher>());
    std::shared_ptr<AbstractCreator> b(new Creator<Course>());
    std::shared_ptr<AbstractCreator> c(new Creator<Student>());
}

TEST(Creators, CallCreate) {
    {
        std::shared_ptr<AbstractCreator> a(new Creator<Teacher>());
        EXPECT_NO_THROW(auto b = a->create("b", 1, 1, vector_ids()));
    }
    {
        std::shared_ptr<AbstractCreator> a(new Creator<Course>());
        std::pair<std::string, std::shared_ptr<AbstractCreator>> p{"course", a};
        EXPECT_NO_THROW(auto b = p.second->create("b", 1, 1, vector_ids()));
    }
    {
        std::shared_ptr<AbstractCreator> a(new Creator<Student>());
        auto a_casted = std::dynamic_pointer_cast<Creator<Student>>(a);
        EXPECT_NO_THROW(auto b = a_casted->create("b", 1, 1, vector_ids()));
    }
}

TEST(Creators, Map) {
    std::shared_ptr<AbstractCreator> a(new Creator<Teacher>());
    std::shared_ptr<AbstractCreator> b(new Creator<Course>());
    std::shared_ptr<AbstractCreator> c(new Creator<Student>());
    std::map<std::string, std::shared_ptr<AbstractCreator>> creators = {
        {"a", a},
        {"b", b},
        {"c", c}
    };
    auto& it_a = creators["a"];
    auto it_b = creators.find("b");
    auto& it_c = creators.find("c");
    EXPECT_NO_THROW(auto teacher = it_a->create("TEACHER", 1, 1, vector_ids()));
    EXPECT_NO_THROW(auto course = it_b->second->create("COURSE", 1, 1, vector_ids()));
    EXPECT_NO_THROW(auto student = it_c->second->create("STUDENT", 1, 1, vector_ids()));
}

TEST(FactoryTest, InvalidTypeOfItem) {
    Factory f;
    EXPECT_ANY_THROW(f.create("qu", "ccc", 10, 1, vector_ids()));
}

TEST(FactoryTest, SimpleObjects) {
    Factory f;
    vector_sh_ptrs teachers;
    vector_sh_ptrs courses;
    vector_sh_ptrs students;
    for (size_t i = 0; i < 10; i++) {
        teachers.push_back(f.create("teacher", "qwe", i, i + 2, vector_ids()));
        courses.push_back(f.create("course", "ccc", i + 10, i, vector_ids()));
        students.push_back(f.create("student", "sss", i + 20, i + 100, vector_ids()));
    }
    EXPECT_EQ(teachers.size(), 10);
    EXPECT_EQ(courses.size(), 10);
    EXPECT_EQ(students.size(), 10);
    EXPECT_ANY_THROW(f.create("qu", "ccc", 10, 1, vector_ids()));
    for (size_t i = 0; i < 10; i++) {
        {
            auto t = std::dynamic_pointer_cast<Teacher>(teachers[i]);
            EXPECT_EQ(t->getName(), "qwe");
            EXPECT_EQ(t->getID(), i);
            EXPECT_EQ(t->getExperience(), i + 2);
            EXPECT_EQ(t->getIDs(), vector_ids());
        }
        {
            auto t = std::dynamic_pointer_cast<Course>(courses[i]);
            EXPECT_EQ(t->getName(), "ccc");
            EXPECT_EQ(t->getID(), i + 10);
            EXPECT_EQ(t->getTeacherID(), i);
            EXPECT_EQ(t->getIDs(), vector_ids());
        }
        {
            auto t = std::dynamic_pointer_cast<Student>(students[i]);
            if (t) {
                EXPECT_EQ(t->getName(), "sss");
                EXPECT_EQ(t->getID(), i + 20);
                EXPECT_EQ(t->getIDs(), vector_ids{i + 100});
            }
        }
    }
}

TEST(Streams, DataBaseFillerInvalid) {
    std::istringstream ss{
        "type2\n"
        ".././asdfg\n"
        "teacher Bob 2 1 12\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    auto expect = std::shared_ptr<Teacher>(new Teacher("Bob", 2, 1, vector_ids{12}));
    auto empty = std::make_pair<std::string, std::string>("", "");
    auto teacher = filler.getNextItem();
    auto empty_item = filler.getNextItem();
    auto t = std::dynamic_pointer_cast<Teacher>(teacher);
    EXPECT_EQ(t->getName(), expect->getName());
    EXPECT_EQ(t->getID(), expect->getID());
    EXPECT_EQ(t->getExperience(), expect->getExperience());
    EXPECT_EQ(t->getIDs(), expect->getIDs());
    EXPECT_EQ(empty_item, nullptr);
}

TEST(Streams, DataBaseFillerRight) {
    std::istringstream ss{
        "teacher A 2 1 12\n"
        "student B 1 1 12 13 14\n"
        "course C 12 2 1\n"
        "student D 3 1 12 14\n"
        "student E 4 1 12 14 15\n"
        "teacher F 5 5 20 21\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    auto expect1 = std::shared_ptr<Teacher>(new Teacher("A", 2, 1, vector_ids{12}));
    auto expect2 = std::shared_ptr<Student>(new Student("B", 1, 1, vector_ids{12, 13, 14}));
    auto expect3 = std::shared_ptr<Course>(new Course("C", 12, 2, vector_ids{1}));
    auto expect4 = std::shared_ptr<Student>(new Student("D", 3, 1, vector_ids{12, 14}));
    auto expect5 = std::shared_ptr<Student>(new Student("E", 4, 1, vector_ids{12, 14, 15}));
    auto expect6 = std::shared_ptr<Teacher>(new Teacher("F", 5, 5, vector_ids{20, 21}));
    auto A = filler.getNextItem();
    auto B = filler.getNextItem();
    auto C = filler.getNextItem();
    auto D = filler.getNextItem();
    auto E = filler.getNextItem();
    auto F = filler.getNextItem();
    EXPECT_EQ(A->getName(), expect1->getName());
    EXPECT_EQ(A->getID(), expect1->getID());
    EXPECT_EQ(A->getIDs(), expect1->getIDs());
    EXPECT_EQ(B->getName(), expect2->getName());
    EXPECT_EQ(B->getID(), expect2->getID());
    EXPECT_EQ(B->getIDs(), expect2->getIDs());
    EXPECT_EQ(C->getName(), expect3->getName());
    EXPECT_EQ(C->getID(), expect3->getID());
    EXPECT_EQ(C->getIDs(), expect3->getIDs());
    EXPECT_EQ(D->getName(), expect4->getName());
    EXPECT_EQ(D->getID(), expect4->getID());
    EXPECT_EQ(D->getIDs(), expect4->getIDs());
    EXPECT_EQ(E->getName(), expect5->getName());
    EXPECT_EQ(E->getID(), expect5->getID());
    EXPECT_EQ(E->getIDs(), expect5->getIDs());
    EXPECT_EQ(F->getName(), expect6->getName());
    EXPECT_EQ(F->getID(), expect6->getID());
    EXPECT_EQ(F->getIDs(), expect6->getIDs());
}

TEST(DataBaseTest, CreateAndDestroy) {
    std::istringstream ss{
        "teacher A 2 1 12\n"
        "student B 1 1 12 13 14\n"
        "course C 12 2 1\n"
        "student D 3 1 12 14\n"
        "student E 4 1 12 14 15\n"
        "teacher F 5 5 20 21\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    DataBase DB;
    DB.fill(filler);
}

TEST(DataBaseTest, AddItemAfterUpdate) {
    std::istringstream ss{
        "teacher A 2 1 12\n"
        "student B 1 4 12 13 14\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    DataBase DB;
    DB.fill(filler);
    DB.update();
    auto item = std::shared_ptr<Item>(new Teacher("A", 2, 1, vector_ids{12}));
    EXPECT_ANY_THROW(DB.addItem(item));
}

TEST(DataBaseTest, UpdateAndGetItems) {
    std::istringstream ss{
        "teacher A 200 1 12\n"
        "student B 1 18 12 13 14\n"
        "course C 12 200 1\n"
        "student D 3 18 12 14\n"
        "student E 4 18 12 14 15\n"
        "teacher F 500 7 20 21\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    DataBase DB;
    DB.fill(filler);
    DB.update();
    auto expect1 = std::shared_ptr<Teacher>(new Teacher("A", 200, 1, vector_ids{12}));
    auto expect2 = std::shared_ptr<Student>(new Student("B", 1, 18, vector_ids{12, 13, 14}));
    auto expect3 = std::shared_ptr<Course>(new Course("C", 12, 200, vector_ids{1}));
    auto expect4 = std::shared_ptr<Student>(new Student("D", 3, 18, vector_ids{12, 14}));
    auto expect5 = std::shared_ptr<Student>(new Student("E", 4, 18, vector_ids{12, 14, 15}));
    auto expect6 = std::shared_ptr<Teacher>(new Teacher("F", 500, 7, vector_ids{20, 21}));
    auto A = DB.getTeacher(200);
    auto B = DB.getStudent(1);
    auto C = DB.getCourse(12);
    auto D = DB.getStudent(3);
    auto E = DB.getStudent(4);
    auto F = DB.getTeacher(500);
    EXPECT_EQ(A->getName(), expect1->getName());
    EXPECT_EQ(A->getID(), expect1->getID());
    EXPECT_EQ(A->getIDs(), expect1->getIDs());
    EXPECT_EQ(B->getName(), expect2->getName());
    EXPECT_EQ(B->getID(), expect2->getID());
    EXPECT_EQ(B->getIDs(), expect2->getIDs());
    EXPECT_EQ(C->getName(), expect3->getName());
    EXPECT_EQ(C->getID(), expect3->getID());
    EXPECT_EQ(C->getIDs(), expect3->getIDs());
    EXPECT_EQ(D->getName(), expect4->getName());
    EXPECT_EQ(D->getID(), expect4->getID());
    EXPECT_EQ(D->getIDs(), expect4->getIDs());
    EXPECT_EQ(E->getName(), expect5->getName());
    EXPECT_EQ(E->getID(), expect5->getID());
    EXPECT_EQ(E->getIDs(), expect5->getIDs());
    EXPECT_EQ(F->getName(), expect6->getName());
    EXPECT_EQ(F->getID(), expect6->getID());
    EXPECT_EQ(F->getIDs(), expect6->getIDs());
}

TEST(Items, Serialize) {
    auto a = std::shared_ptr<Course>(new Course("a", 1, 2, vector_ids{11}));
    auto b = std::shared_ptr<Teacher>(new Teacher("b", 2, 1, vector_ids{12, 13, 14}));
    auto c = std::shared_ptr<Student>(new Student("c", 3, 1, vector_ids{13, 1232}));
    std::ostringstream oss_a;
    std::ostringstream oss_b;
    std::ostringstream oss_c;
    a->serialize(oss_a);
    b->serialize(oss_b);
    c->serialize(oss_c);
    EXPECT_EQ(oss_a.str(), "course\ta\t1\t2\n");
    EXPECT_EQ(oss_b.str(), "teacher\tb\t2\t1\n");
    EXPECT_EQ(oss_c.str(), "student\tc\t3\n");
}

TEST(DataBaseTest, SerializeAll) {
    std::istringstream ss{
        "teacher A 2 1 12\n"
        "student B 1 4 12 13 14\n"
        "course C 12 2 1\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    DataBase DB;
    DB.fill(filler);
    std::ostringstream out;
    DB.serialize(out);
    EXPECT_EQ(out.str(), "course\tC\t12\t2\t1\nteacher\tA\t2\t1\t12\nstudent\tB\t1\t12\n");
}

TEST(DataBaseTest, SerializeOnlyTeacher) {
    std::istringstream ss{
        "asd //sdasd\n"
        "teacher A 2 1 12\n"
        "asd B 3 14\n"
        "fddf C 12 2 1\n"};
    Factory f;
    DataBaseFiller filler(ss, f);
    DataBase DB;
    DB.fill(filler);
    DB.update();
    std::ostringstream out;
    DB.serialize(out);
    EXPECT_EQ(out.str(), "teacher\tA\t2\t1\n");
}
