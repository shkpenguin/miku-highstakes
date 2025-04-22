#pragma once

#include "utils/pose.h"
#include <vector>

struct Waypoint {
    float x;
    float y;
    float dx;
    float dy;
    float ddx;
    float ddy;
    float theta;
    float linvel;
    float angvel;
    float t;
};

Point getDerivative(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = 3 * u * u;
    float b1 = 6 * u * t;
    float b2 = 3 * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2;
}

Point getSecondDerivative(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = 6 * u;
    float b1 = 6 * t;
    return p0 * b0 + p1 * b1;
}

float getCurvature(Point p0, Point p1, Point p2, Point p3, float t) {
    Point d = getDerivative(p0, p1, p2, p3, t);
    Point dd = getSecondDerivative(p0, p1, p2, p3, t);
    return (d.x * dd.y - d.y * dd.x) / pow(d.x * d.x + d.y * d.y, 1.5);
}

class Path {

    public:
        Path() = default;
        Path(std::vector<Waypoint> waypoints) : waypoints(waypoints) {}
        Path(const Path& other) : waypoints(other.waypoints) {}
        Path& operator=(const Path& other) {
            if (this != &other) {
                waypoints = other.waypoints;
            }
            return *this;
        }

        std::vector<Waypoint> waypoints;
};