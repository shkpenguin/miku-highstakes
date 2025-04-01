#pragma once

#include "api.h"

extern pros::MotorGroup left_dt;
extern pros::MotorGroup right_dt;

extern pros::Controller master;

extern pros::adi::DigitalOut clamp;
// extern pros::adi::DigitalOut pistake;
extern pros::Distance dist;

extern pros::Motor intake;
extern pros::Imu imu;

extern pros::MotorGroup lb;