#pragma once

#include <iostream>

class Point final {
private:
    size_t  id;
    int  x, y;

public:
    Point() : id(0), x(0), y(0) {};
    Point(size_t _id, int _x, int _y) : id(_id), x(_x), y(_y) {};
    size_t  getID() const { return id; }
    int getX() const { return x; }
    int getY() const { return y; }
    bool operator==(const Point& other_point) const {
        return id == other_point.id
            && x == other_point.x
            && y == other_point.y;
    }
    
    friend std::ostream& operator<<(std::ostream & os, const Point & point) {
        os << point.id << ": " << point.x << ", " << point.y;
        return os;
    }
};
