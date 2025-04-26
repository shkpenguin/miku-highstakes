#pragma once

#include "chassis/chassis.h"
#include "api.h"
#include "utils/mp.h"

extern pros::MotorGroup left_dt;
extern pros::MotorGroup right_dt;

extern pros::Controller master;

extern pros::adi::DigitalOut clamp;
// extern pros::adi::DigitalOut pistake;
extern pros::adi::DigitalOut hang;
extern pros::adi::DigitalOut pto;

extern pros::Motor intake;
extern pros::Imu imu;

extern pros::MotorGroup lb;
extern pros::Rotation lbRot;

extern pros::Rotation hori;
extern pros::Rotation vertical;

extern pros::Optical optical;

extern pros::Distance leftDist;
extern pros::Distance rightDist;

extern Chassis miku;