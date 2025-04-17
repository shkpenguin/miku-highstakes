#pragma once

#include "chassis.h"
#include "api.h"

#define DT_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 2.75
#define TRACKING_WHEEL_OFFSET 0.5

extern pros::MotorGroup left_dt;
extern pros::MotorGroup right_dt;

extern pros::Controller master;

extern pros::adi::DigitalOut clamp;
// extern pros::adi::DigitalOut pistake;
extern pros::Distance dist;

extern pros::Motor intake;
extern pros::Imu imu;

extern pros::MotorGroup lb;

extern pros::Rotation hori;

extern Chassis miku;