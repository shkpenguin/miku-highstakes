#pragma once

class Pose {
    public:
        float x;
        float y;
        float theta;
        Pose(float x, float y, float theta);
        Pose(float x, float y);
        Pose operator+(const Pose& other) const;
        Pose operator-(const Pose& other) const;
        float operator*(const Pose& other) const;
        Pose operator*(const float& other) const;
        Pose operator/(const float& other) const;
        Pose lerp(Pose other, float t) const;
        float distance(Pose other) const;
        float angle(Pose other) const;
        Pose rotate(float angle) const;
};

class Point {
    public:
        float x;
        float y;
        Point(float x, float y);
        Point operator+(const Point& other) const;
        Point operator-(const Point& other) const;
        float operator*(const Point& other) const;
        Point operator*(const float& other) const;
        Point operator/(const float& other) const;
        float distance(Point other) const;
};