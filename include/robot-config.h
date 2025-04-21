#pragma once

#include "chassis/chassis.h"
#include "api.h"

#define DT_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 2.75
#define VERTICAL_OFFSET 0.0
#define HORIZONTAL_OFFSET -0.5
#define HORIZONTAL_DRIFT 2.0
#define DEADBAND 3
#define MIN_SPEED 10
#define EXPO_CURVE_GAIN 1.019

extern pros::MotorGroup left_dt;
extern pros::MotorGroup right_dt;

extern pros::Controller master;

extern pros::adi::DigitalOut clamp;
// extern pros::adi::DigitalOut pistake;
extern pros::adi::DigitalOut hang;
extern pros::adi::DigitalOut pto;
extern pros::Distance dist;

extern pros::Motor intake;
extern pros::Imu imu;

extern pros::MotorGroup lb;
extern pros::Rotation lbRot;

extern pros::Rotation hori;
extern pros::Rotation vertical;

extern Chassis miku;