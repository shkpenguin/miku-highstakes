#include "utils/pose.h"
#include "utils/utils.h"

Pose::Pose() {
    x = 0;
    y = 0;
    theta = 0;
}

Pose::Pose(float x, float y, float theta) : x(x), y(y), theta(theta) {}

Pose Pose::operator+(const Pose& other) const {
    return Pose(x + other.x, y + other.y, theta + other.theta);
}

Pose Pose::operator-(const Pose& other) const {
    return Pose(x - other.x, y - other.y, theta - other.theta);
}

float Pose::operator*(const Pose& other) const {
    return x * other.x + y * other.y + theta * other.theta;
}

Pose Pose::operator*(const float& other) const {
    return Pose(x * other, y * other, theta * other);
}

Pose Pose::operator/(const float& other) const {
    return Pose(x / other, y / other, theta / other);
}

Pose Pose::lerp(Pose other, float t) const {
    return Pose(x + (other.x - x) * t,
                y + (other.y - y) * t,
                theta + (other.theta - theta) * t);
}

float Pose::distance(Pose other) const {
    return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
}

float Pose::angle(Pose other) const {
    return atan2(other.y - y, other.x - x);
}

Pose Pose::rotate(float angle) const {
    return Pose(x * cos(angle) - y * sin(angle),
                x * sin(angle) + y * cos(angle),
                theta);
}

Point::Point() {
    x = 0;
    y = 0;
}

Point::Point(double x, double y) : x(x), y(y) {}

Point Point::operator+(const Point& other) const {
    return Point(x + other.x, y + other.y);
}

Point Point::operator-(const Point& other) const {
    return Point(x - other.x, y - other.y);
}

double Point::operator*(const Point& other) const {
    return x * other.x + y * other.y;
}

Point Point::operator*(const double& other) const {
    return Point(x * other, y * other);
}

Point Point::operator/(const double& other) const {
    return Point(x / other, y / other);
}

double Point::distance(Point other) const {
    return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
}