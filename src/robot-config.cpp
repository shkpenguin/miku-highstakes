#include "api.h"
#include "robot-config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor intake(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::MotorGroup lb({19, -20}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Rotation lbRot(12);

pros::Imu imu(7);
// pros::Distance dist(21);
pros::adi::DigitalOut hang('A');
// pros::adi::DigitalOut pistake('H');
pros::adi::DigitalOut clamp('B');
pros::adi::DigitalOut pto('G');

pros::MotorGroup left_dt({-1, -6, -4});
pros::MotorGroup right_dt({8, 9, 10});

// pros::Optical optical(20);

pros::Rotation hori(-18);
pros::Rotation vertical(17);

ControllerSettings lateralSettings(0.5, 0.0, 0.0, 0.0, 1.5, 0.0, 2.5, 0.0, 10);

ControllerSettings angularSettings(0.5, 0.0, 0.0, 0.0, 1.5, 0.0, 2.5, 0.0, 10);

Constraints constraints(MAX_VEL, MAX_VEL * 3, 0.1, MAX_VEL * 3, MAX_VEL * 100, TRACK_WIDTH);

ProfileGenerator profileGenerator(&constraints);

Chassis miku(&left_dt, &right_dt, &vertical, &hori, lateralSettings, angularSettings);