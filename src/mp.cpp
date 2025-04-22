#include "mp.h"
#include "utils/utils.h"
#include "utils/math.h"

#include <iostream>

double bezierLength(Point p0, Point p1, Point p2, Point p3, int steps = 100) {
    double length = 0.0;
    Point prev = getPoint(p0, p1, p2, p3, 0.0);
    for (int i = 1; i <= steps; ++i) {
        double t = (double)i / steps;
        Point curr = getPoint(p0, p1, p2, p3, t);
        length += dist(prev.x, prev.y, curr.x, curr.y);
        prev = curr;
    }
    return length;
}



Point getPoint(Point p0, Point p1, Point p2, Point p3, float t) {
    float u = 1 - t;
    float b0 = u * u * u;
    float b1 = 3 * u * u * t;
    float b2 = 3 * u * t * t;
    float b3 = t * t * t;
    return p0 * b0 + p1 * b1 + p2 * b2 + p3 * b3;
}

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

Path& Path::operator=(const Path& other) {
    if (this != &other) {
        waypoints = other.waypoints;
    }
    return *this;
}

void Path::generateWaypoints() {

    // Number of waypoints to generate
    double currentTime = 0;
    float prevX = p0.x;
    float prevY = p0.y;

    // Loop through the control points in sets of 4 (for cubic Bézier interpolation)
    double segmentLength = bezierLength(p0, p1, p2, p3);
    int numWaypoints = segmentLength / 0.5;

    waypoints.clear();
    waypoints.reserve(numWaypoints);

    // Generate waypoints for this segment
    for (int i = 0; i <= numWaypoints; ++i) {
        // Interpolate the current position at parameter t
        float t = (float)i / numWaypoints;
        Point currentPoint = getPoint(p0, p1, p2, p3, t);

        // Calculate derivatives (velocity and acceleration)
        Point velocity = getDerivative(p0, p1, p2, p3, t);
        Point acceleration = getSecondDerivative(p0, p1, p2, p3, t);

        // Calculate curvature (how much the path is turning)
        float curvature = getCurvature(p0, p1, p2, p3, t);

        // Linear velocity (simplified, could use specific logic based on your needs)
        float linearVelocity = velocity.x * velocity.x + velocity.y * velocity.y;
        linearVelocity = std::sqrt(linearVelocity); // Magnitude of velocity vector

        // Angular velocity based on curvature
        float angularVelocity = linearVelocity * curvature;

        // Calculate theta (orientation)
        float theta = std::atan2(velocity.y, velocity.x);

        // Compute time for this waypoint based on velocity
        double deltaTime = dist(prevX, prevY, currentPoint.x, currentPoint.y) / linearVelocity;
        currentTime += deltaTime * 1000; // Convert to milliseconds

        // Create the Waypoint and fill it with data
        Waypoint wp;
        wp.x = currentPoint.x;
        wp.y = currentPoint.y;
        wp.dx = velocity.x;
        wp.dy = velocity.y;
        wp.ddx = acceleration.x;
        wp.ddy = acceleration.y;
        wp.theta = theta;
        wp.linvel = linearVelocity;
        wp.angvel = angularVelocity;
        wp.t = currentTime;

        // Store the waypoint
        waypoints.push_back(wp);

        // Update previous position
        prevX = currentPoint.x;
        prevY = currentPoint.y;
    }
}
