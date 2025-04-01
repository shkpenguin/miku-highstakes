#include "api.h"
#include "robot-config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

// pros::Motor intake(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::MotorGroup lb({14, -15}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

// pros::Imu imu(5);
// pros::Distance dist(21);
pros::adi::DigitalOut clamp('A');
// pros::adi::DigitalOut pistake('H');

pros::MotorGroup left_dt({-1, -2, -3});
pros::MotorGroup right_dt({11, 12, 13});

pros::Motor intake(-4, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

// pros::Optical optical(20);

// pros::Rotation hori(7);