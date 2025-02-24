#pragma once

template<typename T>
class Point {
private:
    T x_;

public:
    Point(const T& x) : x_(x) {}
    T x() const { return x_; }
};

template<typename T>
inline bool operator<(const Point<T>& p1, const Point<T>& p2) { return p1.x() < p2.x(); }

template<typename T>
inline bool operator>(const Point<T>& p1, const Point<T>& p2) { return p1.x() > p2.x(); }
