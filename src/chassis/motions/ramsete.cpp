#include "chassis/chassis.h"
#include "mp.h"
#include "utils/timer.h"
#include "chassis/odom.h"
#include "robot-config.h"

double kV = 1.0;
double kW = 2.0;
double zeta = 0.7;
double b = 2.0;
double dt = 10; // ms

void Chassis::ramsete(Point p0, Point p1, Point p2, Point p3, int timeout) {
    
    this->requestMotionStart();

    if(!this->motionRunning) return;

    lateralLargeExit.reset();
    lateralSmallExit.reset();

    Timer timer(timeout);

    Path p = Path(p0, p1, p2, p3);
    p.generateWaypoints();
    int currentWaypoint = 0;
    int waypointCount = p.waypoints.size();

    Chassis::tbh.setStatus(true);
    // Chassis::tbh.setGain(25.0);

    while(!timer.isDone() && this->motionRunning && (!lateralLargeExit.getExit() && !lateralSmallExit.getExit())) {
        Pose pose = this->getPose();
        double robot_x = pose.x;
        double robot_y = pose.y;
        double robot_theta = pose.theta;

        // --- Find closest waypoint ---
        int closest = currentWaypoint;
        double closest_dist = std::hypot(robot_x - p.waypoints[closest].x, robot_y - p.waypoints[closest].y);

        while (closest + 1 < waypointCount - 1) {
            double next_dist = std::hypot(robot_x - p.waypoints[closest + 1].x, robot_y - p.waypoints[closest + 1].y);
            if (next_dist < closest_dist) {
                closest++;
                closest_dist = next_dist;
            } else {
                break;
            }
        }

        currentWaypoint = closest;

        // --- Pull target waypoint ---
        Waypoint wp = p.waypoints[currentWaypoint];

        // Target pose
        double xt = wp.x;
        double yt = wp.y;
        double thetat = wp.theta;
        double vt = wp.linvel * kV;
        double wt = wp.angvel * kW;

        // Error in robot frame
        double dx = xt - robot_x;
        double dy = yt - robot_y;

        double cosTheta = std::cos(robot_theta);
        double sinTheta = std::sin(robot_theta);

        double error_x = cosTheta * dx + sinTheta * dy;
        double error_y = -sinTheta * dx + cosTheta * dy;
        double error_theta = std::atan2(std::sin(thetat - robot_theta), std::cos(thetat - robot_theta));

        // RAMSETE controller
        double k = 2 * zeta * std::sqrt(wt * wt + b * vt * vt);
        double u1 = -k * error_x;
        double u2 = -b * vt * error_y - k * error_theta;

        if (std::abs(error_theta) > 1e-5) {
            u2 = -b * vt * std::sin(error_theta) / error_theta * error_y - k * error_theta;
        }

        double v = vt * std::cos(error_theta) - u1;
        double w = wt - u2;

        // Convert to wheel speeds
        double leftVel = v - w * (TRACK_WIDTH / 2.0);
        double rightVel = v + w * (TRACK_WIDTH / 2.0);

        tbh.setLeftTarget(leftVel);
        tbh.setRightTarget(rightVel);
        tbh.update();

        pros::delay(dt);
    }

    tbh.setStatus(false);
    left_dt.move_velocity(0);
    right_dt.move_velocity(0);
    
    this->motionRunning = false;
}