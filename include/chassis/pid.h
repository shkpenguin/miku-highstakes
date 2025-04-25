#pragma once

struct Gains {
    float kP;
    float kI;
    float kD;

    Gains(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}
};

class PID {
    public:

        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false, bool trapezoidal = false);

        float update(float error);

        void setGains(Gains gains) {
            kP = gains.kP;
            kI = gains.kI;
            kD = gains.kD;
        }

        void reset();
    protected:
        // gains
        // the gains are unprotected. what i am doing is very smart but also very dangerous. i knew ts would happen
        float kP;
        float kI;
        float kD;

        // optimizations
        const float windupRange;
        const bool signFlipReset;
        const bool trapezoidal;

        float integral = 0;
        float prevError = 0;
};