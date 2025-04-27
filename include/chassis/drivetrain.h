#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include "chassis/pid.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "api.h"

// ts pmo
class Drivetrain {
public:
    Drivetrain();
    Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, ControllerSettings velocitySettings);
    
    void setGains(Gains gains);
    
    double prevLeftTarget = 0; // allows for quick feedforward on high jerk
    double prevRightTarget = 0; 
    void setLeftTarget(double vel);
    void setRightTarget(double vel);
    
    void setLeftVolts(double volts);
    void setRightVolts(double volts);

    void update();

    void reset();

    // protected:
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;

    bool enabled = false;

    PID leftPID;
    PID rightPID;

    float slew;

    double leftTarget = 0;
    double rightTarget = 0;

    double leftVoltage = 0;
    double rightVoltage = 0;
};

double voltageLookup(double vel);