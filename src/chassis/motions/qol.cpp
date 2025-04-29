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
    
}

void Chassis::moveDistanceRaw(float distance, int timeout, MoveDistanceParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([&]() {
            moveDistanceRaw(distance, timeout, params, false);
        });
        this->endMotion();
        pros::delay(10);
        return;
    }
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    const float sign = copysign(1.0f, distance);
    const float initialAngle = vertical.get_position();  // in ticks or degrees
    float prevAngle = initialAngle;
    distTraveled = 0;
    float prevLateralOut = 0;
    Timer timer(timeout);
    bool close = false;

    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) && this->motionRunning) {
        float angleNow = vertical.get_position();
        float deltaAngle = angleNow - prevAngle;
        distTraveled += deg2inch(deltaAngle / 100, TRACKING_WHEEL_DIAMETER);  // Use appropriate conversion
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

    // stop the drivetrain
    drivetrain.reset();
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();

}

void Chassis::moveTime(float time, float speed) {
    this->requestMotionStart();
    if (!this->motionRunning) return;

    drivetrain.setLeftVolts(speed * 120);
    drivetrain.setRightVolts(speed * 120);

    pros::delay(time);

    // stop the drivetrain
    drivetrain.reset();
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}