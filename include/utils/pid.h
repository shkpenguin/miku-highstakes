#pragma once

class PID {
    public:

        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false, bool trapezoidal = false);

        float update(float error);

        void reset();
    protected:
        // gains
        const float kP;
        const float kI;
        const float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;
        const bool trapezoidal;

        float integral = 0;
        float prevError = 0;
};