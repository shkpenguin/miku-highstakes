#include "tbh.h"
#include "robot-config.h"  // For tchassis::right and tchassis::left
#include <algorithm>
#include <cmath>

TBH::TBH(double gain)
  : gain_(gain), enabled_(false) {
}

void TBH::setStatus(bool enabled) {
    enabled_ = enabled;
}

void TBH::setGain(double newGain) {
    gain_ = newGain;
}

void TBH::setLeftTarget(double vel) {
    left_.target = vel;
    left_.tbh = voltageLookup(vel);
    left_.hardTBH = true;
    left_.prevError = 0;
}

void TBH::setRightTarget(double vel) {
    right_.target = vel;
    right_.tbh = voltageLookup(vel);
    right_.hardTBH = true;
    right_.prevError = 0;
}

void TBH::setLeftVolts(double volts) {
    left_.volts = volts;
}

void TBH::setRightVolts(double volts) {
    right_.volts = volts;
}

void TBH::update() {
    if (!enabled_) return;

    // Read current velocities
    double leftVel = avg(left_dt.get_actual_velocity_all());
    double rightVel  = avg(right_dt.get_actual_velocity_all());

    // Compute errors
    right_.error = right_.target - rightVel;
    left_.error  = left_.target  - leftVel;

    // Run TBH for each side
    calculate(right_);
    calculate(left_);

    // Apply voltages
    left_dt.move_voltage(left_.volts);
    right_dt.move_voltage(right_.volts);
}

void TBH::reset() {
    left_.tbh = 0;
    right_.tbh = 0;
    left_.error = 0;
    right_.error = 0;
    left_.prevError = 0;
    right_.prevError = 0;
}

// Linear interpolation: %velocity → feedforward voltage (mV)
double voltageLookup(double vel) {
    // Lookup table: { percent velocity, voltage in millivolts }
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

    // Clamp input to the table range
    vel = std::clamp(vel, -100.0, 100.0);

    // Find the surrounding points for interpolation
    for (int i = 0; i < 24; ++i) {
        double v0 = table[i][0];
        double v1 = table[i + 1][0];
        if (vel >= v0 && vel <= v1) {
            double m0 = table[i][1];
            double m1 = table[i + 1][1];
            double frac = (vel - v0) / (v1 - v0);
            return m0 + frac * (m1 - m0);
        }
    }

    // If we get here, return max/min
    return (vel < 0) ? -12000 : 12000;
}