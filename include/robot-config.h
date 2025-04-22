#pragma once

#include "chassis/chassis.h"
#include "api.h"
#include "mp.h"

// odom settings

#define TRACK_WIDTH 14.0
#define DT_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 2.75
#define VERTICAL_OFFSET 0.0
#define HORIZONTAL_OFFSET -2.5
#define HORIZONTAL_DRIFT 2.0

// mcl

#define HORIZONTAL_DIST_OFFSET 12
#define VERTICAL_DIST_OFFSET 2

// arcade control settings
#define DEADBAND 3
#define MIN_SPEED 10
#define EXPO_CURVE_GAIN 1.019

// ramsete settings
#define K_V 1.0
#define K_W 2.0
#define MAX_VEL 450 * M_PI * 3.25 / 60

extern std::string error_msg;

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

extern pros::Optical optical;

extern Constraints constraints;
extern ProfileGenerator profileGenerator;

extern pros::Distance leftDist;
extern pros::Distance rightDist;

extern Chassis miku;