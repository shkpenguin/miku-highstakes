// tbh_controller.h
#pragma once

#include <cmath>
#include <array>
#include <algorithm>

class TBH {
public:
    TBH(double gain = 25.0);
    
    // Enable or disable TBH loop
    void setStatus(bool enabled);
    
    // Adjust proportional gain
    void setGain(double newGain);
    
    // Set new velocity targets (percent units) for each side
    void setLeftTarget(double vel);
    void setRightTarget(double vel);
    
    // Override voltage directly (in mV)
    void setLeftVolts(double volts);
    void setRightVolts(double volts);

    // Call periodically (e.g., every 10ms)
    void update();

private:
    struct Channel {
        double volts       = 0;
        double tbh         = 0;
        double target      = 0;
        double error       = 0;
        double prevError   = 0;
        bool   hardTBH     = false;
    } left_, right_;

    double gain_;
    bool   enabled_;

    // Maps desired velocity (percent) to feedforward voltage (mV)
    double voltageLookup(double vel);
    
    // Core TBH calculation for one channel
    void calculate(Channel &ch);
};
