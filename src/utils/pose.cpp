#include "utils/pose.h"
#include "utils/utils.h"

class Pose {
    public:
        float x;
        float y;
        float theta;

        Pose(float x, float y, float theta = 0) : x(x), y(y), theta(theta) {}

        Pose operator+(const Pose& other) const {
            return Pose(x + other.x, y + other.y, theta + other.theta);
        }

        Pose operator-(const Pose& other) const {
            return Pose(x - other.x, y - other.y, theta - other.theta);
        }

        float operator*(const Pose& other) const {
            return x * other.x + y * other.y + theta * other.theta;
        }

        Pose operator*(const float& other) const {
            return Pose(x * other, y * other, theta * other);
        }

        Pose operator/(const float& other) const {
            return Pose(x / other, y / other, theta / other);
        }

        Pose lerp(Pose other, float t) const {
            return Pose(x + (other.x - x) * t,
                        y + (other.y - y) * t,
                        theta + (other.theta - theta) * t);
        }

        float distance(Pose other) const {
            return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
        }

        float angle(Pose other) const {
            return atan2(other.y - y, other.x - x);
        }

        Pose rotate(float angle) const {
            return Pose(x * cos(angle) - y * sin(angle),
                        x * sin(angle) + y * cos(angle),
                        theta);
        }
};

class Point {
    public:
        float x;
        float y;

        Point(float x, float y) : x(x), y(y) {}

        Point operator+(const Point& other) const {
            return Point(x + other.x, y + other.y);
        }

        Point operator-(const Point& other) const {
            return Point(x - other.x, y - other.y);
        }

        float operator*(const Point& other) const {
            return x * other.x + y * other.y;
        }

        Point operator*(const float& other) const {
            return Point(x * other, y * other);
        }

        Point operator/(const float& other) const {
            return Point(x / other, y / other);
        }

        float distance(Point other) const {
            return sqrt(pow(other.x - x, 2) + pow(other.y - y, 2));
        }
};