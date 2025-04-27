// The implementation below is mostly based off of
// the document written by Dawgma
// Here is a link to the original document
// https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf

#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "chassis/chassis.h"
#include "utils/utils.h"
#include "robot-config.h"
#include "chassis/odom.h"

/**
 * @brief find the closest point on the path to the robot
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @return int index to the closest point
 */
int findClosest(Pose pose, std::vector<Pose> path) {
    int closestPoint;
    float closestDist = infinity();

    // loop through all path points
    for (int i = 0; i < path.size(); i++) {
        const float dist = pose.distance(path.at(i));
        if (dist < closestDist) { // new closest point
            closestDist = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

/**
 * @brief Function that finds the intersection point between a circle and a line
 *
 * @param p1 start point of the line
 * @param p2 end point of the line
 * @param pos position of the robot
 * @param path the path to follow
 * @return float how far along the line the
 */
float circleIntersect(Pose p1, Pose p2, Pose pose, float lookaheadDist) {
    // calculations
    // uses the quadratic formula to calculate intersection points
    Pose d = p2 - p1;
    Pose f = p1 - pose;
    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookaheadDist * lookaheadDist;
    float discriminant = b * b - 4 * a * c;

    // if a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        // prioritize further down the path
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found
    return -1;
}

/**
 * @brief returns the lookahead point
 *
 * @param lastLookahead - the last lookahead point
 * @param pose - the current position of the robot
 * @param path - the path to follow
 * @param closest - the index of the point closest to the robot
 * @param lookaheadDist - the lookahead distance of the algorithm
 */
Pose lookaheadPoint(Pose lastLookahead, Pose pose, std::vector<Pose> path, int closest,
                            float lookaheadDist) {
    // optimizations applied:
    // only consider intersections that have an index greater than or equal to the point closest
    // to the robot
    // and intersections that have an index greater than or equal to the index of the last
    // lookahead point
    const int start = std::max(closest, int(lastLookahead.theta));
    for (int i = start; i < path.size() - 1; i++) {
        Pose lastPathPose = path.at(i);
        Pose currentPathPose = path.at(i + 1);

        float t = circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

        if (t != -1) {
            Pose lookahead = lastPathPose.lerp(currentPathPose, t);
            lookahead.theta = i;
            return lookahead;
        }
    }

    // robot deviated from path, use last lookahead point
    return lastLookahead;
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the lookahead point
 *
 * @param pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return float curvature
 */
float findLookaheadCurvature(Pose pose, float heading, Pose lookahead) {
    // calculate whether the robot is on the left or right side of the circle
    float side = sgn(std::sin(heading) * (lookahead.x - pose.x) - std::cos(heading) * (lookahead.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(heading);
    float c = std::tan(heading) * pose.x - pose.y;
    float x = std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

void Chassis::pursuit(std::vector<Pose>& path, float lookahead, int timeout, bool forwards, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { pursuit(path, lookahead, timeout, forwards, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    if (path.size() == 0) {
        // set distTraveled to -1 to indicate that the function has finished
        distTraveled = -1;
        // give the mutex back
        this->endMotion();
        return;
    }
    Pose pose = getPose(true);
    Pose lastPose = pose;
    Pose lookaheadPose(0, 0, 0);
    Pose lastLookahead = path.at(0);
    lastLookahead.theta = 0;
    float curvature;
    float targetVel;
    float prevLeftVel = 0;
    float prevRightVel = 0;
    int closestPoint;
    float leftInput = 0;
    float rightInput = 0;
    float prevVel = 0;
    int compState = pros::competition::get_status();
    distTraveled = 0;

    enableCorrection();

    // loop until the robot is within the end tolerance
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState && this->motionRunning; i++) {
        // get the current position of the robot
        pose = getPose(true);
        if (!forwards) pose.theta -= M_PI;

        // update completion vars
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // find the closest point on the path to the robot
        closestPoint = findClosest(pose, path);
        // if the robot is at the end of the path, then stop
        if (path.at(closestPoint).theta == 0) break;

        // find the lookahead point
        lookaheadPose = lookaheadPoint(lastLookahead, pose, path, closestPoint, lookahead);
        lastLookahead = lookaheadPose; // update last lookahead position

        // get the curvature of the arc between the robot and the lookahead point
        float curvatureHeading = M_PI / 2 - pose.theta;
        curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

        // get the target velocity of the robot
        targetVel = path.at(closestPoint).theta;
        targetVel = slew(targetVel, prevVel, lateralSettings.slew);
        prevVel = targetVel;

        // calculate target left and right velocities
        float targetLeftVel = targetVel * (2 + curvature * TRACK_WIDTH) / 2;
        float targetRightVel = targetVel * (2 - curvature * TRACK_WIDTH) / 2;

        // ratio the speeds to respect the max speed
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // update previous velocities
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;

        // move the drivetrain
        if (forwards) {
            drivetrain.setLeftTarget(vel2rpm(targetLeftVel));
            drivetrain.setRightTarget(vel2rpm(targetRightVel));
        } else {
            drivetrain.setRightTarget(-vel2rpm(targetLeftVel));
            drivetrain.setLeftTarget(-vel2rpm(targetRightVel));
        }

        pros::delay(10);
    }

    // stop the robot
    drivetrain.reset();
    disableCorrection();
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    // give the mutex back
    this->endMotion();
}