#include "api.h"
#include "robot-config.h"
#include "chassis/pid.h"

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

// positional gain scheduling, made obsolete by PID velocity control
/*
std::vector<Gains> lateralGains = {
    Gains(8, 0, 20), // no goal
    Gains(8, 0, 20), // empty goal
    Gains(8, 0, 20), // 1 ring
    Gains(8, 0, 20), // 2 rings
    Gains(8, 0, 20), // 3 rings
    Gains(8, 0, 20), // 4 rings
    Gains(8, 0, 20), // 5 rings
    Gains(8, 0, 20), // 6 rings
    Gains(8, 0, 20), // full goal
};
*/

/*
std::vector<Gains> angularGains = {
    Gains(4, 0.15, 24), // no goal
    Gains(4, 0.15, 24), // empty goal
    Gains(4, 0.15, 24), // 1 ring
    Gains(4, 0.15, 24), // 2 rings
    Gains(4, 0.15, 24), // 3 rings
    Gains(4, 0.15, 24), // 4 rings
    Gains(4, 0.15, 24), // 5 rings
    Gains(4, 0.15, 24), // 6 rings
    Gains(4, 0.15, 24) // full goal
};
*/

ControllerSettings velocitySettings(
    0.5, // proportional gain (kP)
    0.0, // integral gain (kI)
    0.0, // derivative gain (kD)
    0.0, // anti windup
    0.0, // unused
    0, // unused
    0.0, // unused
    0, // unused
    0.0, // slew(not coded)
    true // trapezoidal riemann sum
);

ControllerSettings lateralSettings(8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0, // maximum acceleration (slew)
                                              false // no trapezoidal riemann sum
);

ControllerSettings angularSettings(4, // proportional gain (kP)
                                   0.15, // integral gain (kI)
                                   24, // derivative gain (kD)
                                   5, // anti windup
                                   1, // small error range, in inches
                                   100, // small error range timeout, in milliseconds
                                   3, // large error range, in inches
                                   500, // large error range timeout, in milliseconds
                                   0, // maximum acceleration (slew)
                                   true // trapezoidal riemann sum
);

Drivetrain drivetrain(&left_dt, &right_dt, velocitySettings);

Chassis miku(drivetrain, &vertical, &hori, lateralSettings, angularSettings);