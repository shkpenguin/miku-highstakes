#include "api.h"
#include "robot-config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor intake(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::MotorGroup lb({19, -20}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

pros::Imu imu(7);
// pros::Distance dist(21);
pros::adi::DigitalOut clamp('A');
// pros::adi::DigitalOut pistake('H');

pros::MotorGroup left_dt({-1, -3, -4});
pros::MotorGroup right_dt({8, 9, 10});

// pros::Optical optical(20);

// pros::Rotation hori(7);