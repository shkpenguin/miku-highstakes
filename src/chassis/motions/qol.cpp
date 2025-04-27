#include <cmath>
#include "chassis/chassis.h"
#include "utils/timer.h"
#include "utils/utils.h"
#include "pros/misc.hpp"
#include "chassis/odom.h"
#include "robot-config.h"

void Chassis::moveDistance(float distance, int timeout, MoveDistanceParams params, bool async) {
    bool forwards = sgn(distance) > 0;
    Pose pose = getPose();
    pose.theta = fmod(pose.theta, 360);
    // calculate the target pose
    Pose target = pose;
    target.x += distance * cos(pose.theta);
    target.y += distance * sin(pose.theta);
    target.theta = pose.theta;
    // call movePoint with the target pose
    MovePointParams newParams = {params.maxSpeed, params.minSpeed, params.earlyExitRange, forwards};
    movePoint(target.x, target.y, timeout, newParams, async);
    // set the target pose to the current pose
    target.x = pose.x;
}

void Chassis::moveDistanceRaw(float distance, int timeout, MoveDistanceParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    if (!this->motionRunning) return;

    if (async) {
        float dist = distance;
        MoveDistanceParams copyParams = params;
        pros::Task task([this, dist, timeout, copyParams]() {
            moveDistance(dist, timeout, copyParams, false);
        });
        this->endMotion();
        pros::delay(10);
        return;
    }

    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();

    const float sign = copysign(1.0f, distance);
    const float initialAngle = verticalTracker->get_angle();  // in ticks or degrees
    float prevAngle = initialAngle;
    float distTraveled = 0;
    float prevLateralOut = 0;
    Timer timer(timeout);
    bool close = false;

    while (!timer.isDone() &&
           ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        float angleNow = verticalTracker->get_angle();
        float deltaAngle = angleNow - prevAngle;
        distTraveled += deg2inch(TRACKING_WHEEL_DIAMETER * M_PI, deltaAngle);  // Use appropriate conversion
        prevAngle = angleNow;

        float remaining = distance - distTraveled;

        if (fabs(remaining) < 7.5 && !close) {
            close = true;
            params.maxSpeed = std::max(std::fabs(prevLateralOut), 60.0f);
        }

        lateralSmallExit.update(remaining);
        lateralLargeExit.update(remaining);

        float lateralOut = lateralPID.update(remaining);
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        if (sign > 0 && lateralOut < fabs(params.minSpeed) && lateralOut > 0)
            lateralOut = fabs(params.minSpeed);
        if (sign < 0 && lateralOut > -fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);
        if (!close && lateralOut * sign < 0)
            lateralOut = 0;

        prevLateralOut = lateralOut;

        drivetrain.setLeftVolts(lateralOut * 120);
        drivetrain.setRightVolts(lateralOut * 120);

        pros::delay(10);
        if (fabs(remaining) < params.earlyExitRange) break;
    }

    drivetrain.setLeftVolts(0);
    drivetrain.setRightVolts(0);
    distTraveled = -1;
    this->endMotion();
}

