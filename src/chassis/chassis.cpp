#include "utils/math.h"
#include "robot-config.h"
#include "main.h"
#include "routes.h"
#include "utils/utils.h"
#include "chassis/odom.h"

double Chassis::getTracker(pros::Rotation* tracker) {
    return tracker->get_position();
}

Chassis::Chassis(Drivetrain drivetrain,
                 pros::Rotation* verticalTracker, pros::Rotation* horiTracker,
                 ControllerSettings lateralSettings, ControllerSettings angularSettings) :
    drivetrain(drivetrain),
    verticalTracker(verticalTracker),
    horiTracker(horiTracker),
    lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, true),
    angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
    lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
    lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
    angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
    angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

void Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void Chassis::waitUntilDone() {
    while (distTraveled != -1) {
        pros::delay(10);
    }
}

double Chassis::get_velocity(pros::MotorGroup* motors) {
    return avg(motors->get_actual_velocity_all());
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
    float theta = getPose().theta;
    setPose(Pose(0, 0, theta), false);
}

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    left_dt.set_brake_mode_all(mode);
    right_dt.set_brake_mode_all(mode);
}

void Chassis::arcade(int throttle, int turn, bool disableDriveCurve, float desaturateBias) {

    if (!disableDriveCurve) {
        throttle = std::round(curve(throttle, DEADBAND, EXPO_CURVE_GAIN, MIN_SPEED));
        turn = std::round(curve(turn, DEADBAND, EXPO_CURVE_GAIN, MIN_SPEED));
    }
    // desaturate motors based on joyBias
    if (std::abs(throttle) + std::abs(turn) > 127) {
        int oldThrottle = throttle;
        int oldTurn = turn;
        throttle *= (1 - desaturateBias * std::abs(oldTurn / 127.0));
        turn *= (1 - (1 - desaturateBias) * std::abs(oldThrottle / 127.0));
        // ensure the sum of the two values is equal to 127
        // this check is necessary because of integer division
        if (std::abs(turn) + std::abs(throttle) == 126) {
            if (desaturateBias < 0.5) throttle += sgn(throttle);
            else turn += sgn(turn);
        }
    }

    int leftPower = throttle + turn;
    int rightPower = throttle - turn;

    // move drive
    // drivetrain.setLeftVolts(leftPower);
    // drivetrain.setRightVolts(rightPower);
    left_dt.move(leftPower);
    right_dt.move(rightPower);

}