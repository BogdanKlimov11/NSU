#include <cmath>

#include "vector.hpp"

#define _USE_MATH_DEFINES

namespace {
    const double EPSILON = 1e-7;
}

Vector::Vector(void) {
    X = 0;
    Y = 0;
}

Vector::Vector(double x, double y) {
    X = x;
    Y = y;
}

Vector::Vector(const Vector& that) {
    X = that.X;
    Y = that.Y;
}

Vector& Vector::operator=(const Vector& that) = default;

Vector Vector::makePolar(double rad, double alpha) {
    Vector polar_vector;
    polar_vector.X = rad * cos(alpha);
    polar_vector.Y = rad * sin(alpha);
    return polar_vector;
}

double Vector::x(void) const {
    return X;
}

double Vector::y(void) const {
    return Y;
}

void Vector::x(double newX) {
    X = newX;
}

void Vector::y(double newY) {
    Y = newY;
}

Vector Vector::operator+(const Vector& that) const {
    return Vector(X + that.X, Y + that.Y);
}

Vector Vector::operator-(const Vector& that) const {
    return Vector(X - that.X, Y - that.Y);
}

double Vector::operator*(const Vector& that) const {
    double result = X * that.X + Y * that.Y;
    return result;
}

Vector Vector::operator*(const double& that) const {
    return Vector(that * X, that * Y);
}

Vector Vector::operator/(const double& that) const {
    return Vector(X / that, Y / that);
}

Vector& Vector::operator+=(const Vector& that) {
    X = X + that.X;
    Y = Y + that.Y;
    return *this;
}

Vector& Vector::operator-=(const Vector& that) {
    X = X - that.X;
    Y = Y - that.Y;
    return *this;
}

Vector& Vector::operator*=(const double& that) {
    X = X * that;
    Y = Y * that;
    return *this;
}

Vector& Vector::operator/=(const double& that) {
    X = X / that;
    Y = Y / that;
    return *this;
}

Vector Vector::operator-() {
    X = -X;
    Y = -Y;
    return *this;
}

bool Vector::operator==(const Vector& that) const {
    return (abs(X - that.X) <= EPSILON && abs(Y - that.Y) <= EPSILON);
}

bool Vector::operator!=(const Vector& that) const {
    if (this->operator==(that))
        return false;
    else
        return true;
}

Vector& Vector::rotate(double angle) {
    double tempX = X;
    double tempY = Y;
    X = tempX * cos(angle) - tempY * sin(angle);
    Y = tempX * sin(angle) + tempY * cos(angle);
    if (abs(X) <= EPSILON)
        X = 0;
    if (abs(Y) <= EPSILON)
        Y = 0;
    return *this;
}

Vector& Vector::rotate(int quad) {
    double tempX = X;
    double tempY = Y;
    X = tempX * cos(M_PI * quad / 2) - tempY * sin(M_PI * quad / 2);
    Y = tempX * sin(M_PI * quad / 2) + tempY * cos(M_PI * quad / 2);
    if (abs(X) <= EPSILON)
        X = 0;
    if (abs(Y) <= EPSILON)
        Y = 0;
    return *this;
}

double Vector::module2(void) const {
    double module2 = X * X + Y * Y;
    return module2;
}

double Vector::angle(void) const {
    if (abs(X) <= EPSILON) {
        if (abs(Y) <= EPSILON) {
            return 0;
        } else if (Y > 0) {
            return M_PI / 2;
        } else if (Y < 0) {
            return -M_PI / 2;
        }
    }
    double angle = atan2(Y, X);
    return angle;
}

double Vector::angle(const Vector& that) const {
    if (*this == that) {
        return 0;
    }
    double module_a = sqrt(X * X + Y * Y);
    double module_b = sqrt(that.X * that.X + that.Y * that.Y);
    if (*this == that) {
        return 0;
    }
    double scalar_product = *this * that;
    double sgn = 1.0;
    double angle_this = this->angle();
    double angle_that = that.angle();
    if (angle_this < (angle_that - EPSILON)) {
        sgn = -sgn;
    }
    double angle = sgn * acos(scalar_product / (module_a * module_b));
    return angle;
}

double Vector::projection(const Vector& base) const {
    double module_b = sqrt(base.X * base.X + base.Y * base.Y);
    double scalar_product = X * base.X + Y * base.Y;
    double projection = scalar_product / module_b;
    return projection;
}

Vector& Vector::normalize(void) {
    this->operator/=(sqrt(this->module2()));
    return *this;
}

Vector& Vector::transformTo(const Vector& e1, const Vector& e2) {
    double det = e1.X * e2.Y - e2.X * e1.Y;
    double tempX = X;
    double tempY = Y;
    X = (e2.Y * tempX - e2.X * tempY) / det;
    Y = (-e1.Y * tempX + e1.X * tempY) / det;
    return *this;
}

Vector& Vector::transformFrom(const Vector& e1, const Vector& e2) {
    double tempX = X;
    double tempY = Y;
    X = e1.X * tempX + e2.X * tempY;
    Y = e1.Y * tempX + e2.Y * tempY;
    return *this;
}

double Vector::length(void) const {
    return sqrt(this->module2());
}

Vector operator*(const double& lhs, const Vector& rhs) {
    double x = lhs * rhs.x();
    double y = lhs * rhs.y();
    return Vector(x, y);
}

Vector rotate(const Vector& v, double angle) {
    double x = v.x() * cos(angle) - v.y() * sin(angle);
    double y = v.y() * cos(angle) + v.x() * sin(angle);
    return Vector(x, y);
}

Vector rotate(const Vector& v, int quad) {
    double x = v.x() * cos(M_PI * quad / 2) - v.y() * sin(M_PI * quad / 2);
    double y = v.y() * cos(M_PI * quad / 2) + v.x() * sin(M_PI * quad / 2);
    return Vector(x, y);
}

double module2(const Vector& v) {
    return v.module2();
}

double angle(const Vector& v) {
    return v.angle();
}

double angle(const Vector& v1, const Vector& v2) {
    double phi = v1.operator*(v2) / sqrt(v1.module2() * v2.module2());
    return phi;
}

double projection(const Vector& v, const Vector& base) {
    double projection = v.operator*(base) / sqrt(base.module2());
    return projection;
}

Vector normalize(const Vector& v) {
    return v.operator/(sqrt(v.module2()));
}

Vector transformTo(const Vector& v, const Vector& e1, const Vector& e2) {
    double x = e2.y() * v.x() - e1.y() * v.y();
    double y = -e2.x() * v.x() + e1.y() * v.y();
    return Vector(x, y);
}

Vector transformFrom(const Vector& v, const Vector& e1, const Vector& e2) {
    double x = e1.x() * v.x() + e2.x() * v.y();
    double y = e1.y() * v.x() + e2.y() * v.y();
    return Vector(x, y);
}

double length(const Vector& v) {
    return sqrt(v.module2());
}
