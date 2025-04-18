#include "utils/math.h"
#include "robot-config.h"
#include "main.h"
#include "routes.h"
#include "utils/utils.h"

void panic() {
    panicMode = true;
}

void calibrateIMU() {
    int attempt = 1;
    while (attempt <= 3) {
        imu.reset();
        while(imu.is_calibrating()) {
            pros::delay(10);
            if(imu.get_status() == pros::ImuStatus::error) {
                master.rumble("---");
                error_msg = "imu is fucked";
                break;
            }
        }
        if(imu.get_status() == pros::ImuStatus::ready) {
            master.rumble(".");
            break;
        }
        attempt++;
    }
    if (attempt > 3) {
        error_msg = "kill yourself";
        panic();
    }
}

void Chassis::setPose(float x, float y, float theta, bool radians) {
    setPose(Pose(x, y, theta), radians);
}

void Chassis::setPose(Pose pose, bool radians) { setPose(pose, radians); }

Pose Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = rad2deg(pose.theta);
    return pose;
}

void Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool Chassis::isInMotion() const { return this->motionRunning; }

void Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    setPose(Pose(0, 0, theta), false);
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    left_dt.set_brake_mode_all(mode);
    right_dt.set_brake_mode_all(mode);
}