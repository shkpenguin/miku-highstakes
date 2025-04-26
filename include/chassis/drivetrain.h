#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include "chassis/pid.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "api.h"

class Drivetrain {
public:
    Drivetrain();
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, ControllerSettings velocitySettings);

    // Enable or disable auto voltage control
    void setAuto(bool _enabled);
    
    void setGains(Gains gains);
    
    void setLeftTarget(double vel);
    void setRightTarget(double vel);
    
    void setLeftVolts(double volts);
    void setRightVolts(double volts);

    void update();

    void reset();

    // protected:
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;

    bool enabled = true;

    PID leftPID;
    PID rightPID;

    float slew;

    double leftTarget = 0;
    double rightTarget = 0;

    double leftVoltage = 0;
    double rightVoltage = 0;
};

double voltageLookup(double vel);