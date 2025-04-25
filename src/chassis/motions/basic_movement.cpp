#include <cmath>
#include "chassis/chassis.h"
#include "utils/timer.h"
#include "utils/utils.h"
#include "pros/misc.hpp"
#include "chassis/odom.h"
#include "robot-config.h"

void Chassis::moveDistance(float distance, int timeout, MoveDistanceParams params, bool async) {
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
    const float initialAngle = vertical.get_angle();  // in ticks or degrees
    float prevAngle = initialAngle;
    float distTraveled = 0;
    float prevLateralOut = 0;
    Timer timer(timeout);
    bool close = false;

    while (!timer.isDone() &&
           ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        float angleNow = vertical.get_angle();
        float deltaAngle = angleNow - prevAngle;
        distTraveled += deg2inch(TRACKING_WHEEL_DIAMETER * M_PI, deltaAngle);  // Use appropriate conversion
        prevAngle = angleNow;

        float remaining = distance - distTraveled;
        pros::lcd::set_text(1, "Remaining: " + std::to_string(remaining));
        pros::lcd::set_text(2, "Traveled: " + std::to_string(distTraveled));

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

        this->leftMotors->move(lateralOut);
        this->rightMotors->move(lateralOut);

        pros::delay(10);
        if (fabs(remaining) < params.earlyExitRange) break;
    }

    this->leftMotors->move(0);
    this->rightMotors->move(0);
    distTraveled = -1;
    this->endMotion();
}

void Chassis::moveTime(float time, float speed) {
    this->requestMotionStart();
    if (!this->motionRunning) return;

    this->leftMotors->move(speed);
    this->rightMotors->move(speed);

    pros::delay(time);

    this->leftMotors->move(0);
    this->rightMotors->move(0);
    this->endMotion();
}

void Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToHeading(theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));

        targetTheta = theta;

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        this->leftMotors->move(motorPower);
        this->rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    this->leftMotors->move(0);
    this->rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}

void Chassis::turnToPoint(float x, float y, int timeout, TurnToPointParams params, bool async) {
    params.minSpeed = std::abs(params.minSpeed);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { turnToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }
    float targetTheta;
    float deltaX, deltaY, deltaTheta;
    float motorPower;
    float prevMotorPower = 0;
    float startTheta = getPose().theta;
    bool settling = false;
    std::optional<float> prevRawDeltaTheta = std::nullopt;
    std::optional<float> prevDeltaTheta = std::nullopt;
    std::uint8_t compState = pros::competition::get_status();
    distTraveled = 0;
    Timer timer(timeout);
    angularLargeExit.reset();
    angularSmallExit.reset();
    angularPID.reset();

    // main loop
    while (!timer.isDone() && !angularLargeExit.getExit() && !angularSmallExit.getExit() && this->motionRunning) {
        // update variables
        Pose pose = getPose();
        pose.theta = (params.forwards) ? fmod(pose.theta, 360) : fmod(pose.theta - 180, 360);

        // update completion vars
        distTraveled = fabs(angleError(pose.theta, startTheta, false));

        deltaX = x - pose.x;
        deltaY = y - pose.y;
        targetTheta = fmod(rad2deg(M_PI_2 - atan2(deltaY, deltaX)), 360);

        // check if settling
        const float rawDeltaTheta = angleError(targetTheta, pose.theta, false);
        if (prevRawDeltaTheta == std::nullopt) prevRawDeltaTheta = rawDeltaTheta;
        if (sgn(rawDeltaTheta) != sgn(prevRawDeltaTheta)) settling = true;
        prevRawDeltaTheta = rawDeltaTheta;

        // calculate deltaTheta
        if (settling) deltaTheta = angleError(targetTheta, pose.theta, false);
        else deltaTheta = angleError(targetTheta, pose.theta, false, params.direction);
        if (prevDeltaTheta == std::nullopt) prevDeltaTheta = deltaTheta;

        // motion chaining
        if (params.minSpeed != 0 && fabs(deltaTheta) < params.earlyExitRange) break;
        if (params.minSpeed != 0 && sgn(deltaTheta) != sgn(prevDeltaTheta)) break;

        // calculate the speed
        motorPower = angularPID.update(deltaTheta);
        angularLargeExit.update(deltaTheta);
        angularSmallExit.update(deltaTheta);

        // cap the speed
        if (motorPower > params.maxSpeed) motorPower = params.maxSpeed;
        else if (motorPower < -params.maxSpeed) motorPower = -params.maxSpeed;
        if (fabs(deltaTheta) > 20) motorPower = slew(motorPower, prevMotorPower, angularSettings.slew);
        if (motorPower < 0 && motorPower > -params.minSpeed) motorPower = -params.minSpeed;
        else if (motorPower > 0 && motorPower < params.minSpeed) motorPower = params.minSpeed;
        prevMotorPower = motorPower;

        // move the drivetrain
        this->leftMotors->move(motorPower);
        this->rightMotors->move(-motorPower);

        pros::delay(10);
    }

    // stop the drivetrain
    this->leftMotors->move(0);
    this->rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}