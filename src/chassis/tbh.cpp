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
    double rightVel = avg(left_dt.get_actual_velocity_all());
    double leftVel  = avg(right_dt.get_actual_velocity_all());

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

void TBH::calculate(Channel &ch) {
    // Increment voltage by proportional term
    ch.volts += gain_ * ch.error;
    // Clamp output to ±12V (in mV units)  
    if (std::abs(ch.volts) > 12000) {
        ch.volts = std::copysign(12000.0, ch.volts);
    }
    
    // Detect zero-crossing of error
    bool signChange = (ch.prevError > 0) != (ch.error > 0);
    if (signChange) {
        if (ch.hardTBH) {
            // On first zero-cross, use feedforward TBH value
            ch.volts    = ch.tbh;
            ch.hardTBH  = false;
        } else {
            // Subsequent crosses: average with previous TBH
            ch.volts    = 0.5 * (ch.volts + ch.tbh);
            ch.tbh      = ch.volts;
        }
    }
    ch.prevError = ch.error;
}

// Linear interpolation lookup from velocity to feedforward voltage
double TBH::voltageLookup(double vel) {
    static const double table[25][2] = {
        {-12000, -100.0}, {-11000, -91}, {-10000, -82}, {-9000, -73.7},
        {-8000,  -64.4}, {-7000, -55.6}, {-6000, -46.7}, {-5000, -37.3},
        {-4000,  -29.1}, {-3000, -20.5}, {-2000, -11.1}, {-1000,  -3.9},
        {     0,   0.0}, { 1000,   2.7}, { 2000,  10.3}, { 3000,  19.6},
        { 4000,  27.9}, { 5000,  36.1}, { 6000,  45.3}, { 7000,  54.4},
        { 8000,  63.2}, { 9000,  72.6}, {10000,  81.0}, {11000,  90.0},
        {12000, 100.0}
    };
    
    // Estimate index
    int idx = static_cast<int>((vel + 100.0) / 200.0 * 24.0);
    idx = std::clamp(idx, 0, 24);

    // Find interval
    if (table[idx][1] == vel) {
        return table[idx][0];
    }
    int low = idx;
    if (table[low][1] > vel) {
        while (low > 0 && table[low][1] > vel) {
            low--;
        }
    } else {
        while (low < 24 && table[low][1] < vel) {
            low++;
        }
        low--;
    }
    double v0 = table[low][1];
    double v1 = table[low + 1][1];
    double m0 = table[low][0];
    double m1 = table[low + 1][0];
    double frac = (vel - v0) / (v1 - v0);
    return m0 + frac * (m1 - m0);
}