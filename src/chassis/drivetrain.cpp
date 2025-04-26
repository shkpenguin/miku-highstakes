#include "chassis/drivetrain.h"
#include "robot-config.h"  
#include <algorithm>
#include <cmath>

Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, ControllerSettings velocitySettings)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      leftPID(velocitySettings.kP, velocitySettings.kI, velocitySettings.kD, velocitySettings.windupRange,
              velocitySettings.trapezoidal),
      rightPID(velocitySettings.kP, velocitySettings.kI, velocitySettings.kD, velocitySettings.windupRange,
               velocitySettings.trapezoidal),
      slew(velocitySettings.slew) {}

void Drivetrain::setAuto(bool _enabled) {
    enabled = _enabled;
}

void Drivetrain::setGains(Gains gains) {
    leftPID.setGains(gains);
    rightPID.setGains(gains);
}

void Drivetrain::setLeftTarget(double vel) {
    leftTarget = vel;
    leftVoltage = voltageLookup(vel);
    leftPID.reset(); // only resets prevError and integral term
}

void Drivetrain::setRightTarget(double vel) {
    rightTarget = vel;
    rightVoltage = voltageLookup(vel);\
    rightPID.reset();
}

void Drivetrain::setLeftVolts(double volts) {
    leftVoltage = volts;
}

void Drivetrain::setRightVolts(double volts) {
    rightVoltage = volts;
}

void Chassis::updateVoltage() {
    if(drivetrain.enabled) {
        double leftVel = avg(drivetrain.leftMotors->get_actual_velocity_all());
        double rightVel  = avg(drivetrain.rightMotors->get_actual_velocity_all());

        double rightError = drivetrain.rightTarget - rightVel;
        double leftError  = drivetrain.leftTarget  - leftVel;

        drivetrain.leftPID.update(leftError);
        drivetrain.rightPID.update(rightError);
    }

    drivetrain.leftMotors->move_voltage(drivetrain.leftVoltage);
    drivetrain.rightMotors->move_voltage(drivetrain.leftVoltage);
}

void Drivetrain::reset() {
    leftVoltage = 0;
    rightVoltage = 0;
    leftTarget = 0;
    rightTarget = 0;
    leftPID.reset();
    rightPID.reset();
    setAuto(false);
}

double voltageLookup(double vel) {
    // Lookup table: { voltage in millivolts, rpm }
    static const double table[29][2] = {
        {-12000, -640},
        {-11000, -590},
        {-10000, -550},
        {-9000, -490},
        {-8000, -430},
        {-7000, -370},
        {-6000, -310},
        {-5000, -240},
        {-4000, -180},
        {-3000, -130},
        {-2000, -65},
        {-1500, -40},
        {-1000, 0},
        {-500, 0},
        {0, 0},
        {500, 0},
        {1000, 0},
        {1500, 40},
        {2000, 65},
        {3000, 130},
        {4000, 180},
        {5000, 240},
        {6000, 310},
        {7000, 370},
        {8000, 430},
        {9000, 490},
        {10000, 550},
        {11000, 590},
        {12000, 640}
    };

    vel = std::clamp(vel, -640.0, 640.0); 
    for (int i = 0; i < 28; ++i) {
        double rpm0 = table[i][1];
        double rpm1 = table[i + 1][1];

        if ((vel >= rpm0 && vel <= rpm1) || (vel <= rpm0 && vel >= rpm1)) {
            double volt0 = table[i][0];
            double volt1 = table[i + 1][0];
            double frac = (vel - rpm0) / (rpm1 - rpm0);
            return volt0 + frac * (volt1 - volt0);
        }
    }

    return (vel < table[0][1]) ? table[0][0] : table[28][0];
}