#include "api.h"
#include "robot-config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor intake(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);
pros::MotorGroup lb({19, -20}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
pros::Rotation lbRot(14);

pros::Imu imu(7);
// pros::Distance dist(21);
pros::adi::DigitalOut hang('A');
// pros::adi::DigitalOut pistake('H');
pros::adi::DigitalOut clamp('B');
pros::adi::DigitalOut pto('G');

pros::MotorGroup left_dt({-1, -6, -4});
pros::MotorGroup right_dt({8, 9, 10});

pros::Optical optical(13);

pros::Distance leftDist(11);
pros::Distance rightDist(16);

pros::Rotation hori(-18);
pros::Rotation vertical(17);

ControllerSettings lateralSettings(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// ControllerSettings lateralGoalSettings(6, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               20, // derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in inches
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in inches
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

ControllerSettings angularSettings(4, // proportional gain (kP)
                                   0.15, // integral gain (kI)
                                   24, // derivative gain (kD)
                                   5, // anti windup
                                   1, // small error range, in inches
                                   100, // small error range timeout, in milliseconds
                                   3, // large error range, in inches
                                   500, // large error range timeout, in milliseconds
                                   0 // maximum acceleration (slew)
);

Chassis miku(&left_dt, &right_dt, &vertical, &hori, lateralSettings, angularSettings);