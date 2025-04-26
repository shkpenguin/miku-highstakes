#pragma once

/** 
 * @param trapezoidal whether the integral is calculated with a trapezoidal riemann sum. false by default
 */
class ControllerSettings {
    public:
        ControllerSettings() 
            : kP(0), kI(0), kD(0), windupRange(0), smallError(0), smallErrorTimeout(0),
              largeError(0), largeErrorTimeout(0), slew(0), trapezoidal(0) {}

        ControllerSettings(float kP, float kI, float kD, float windupRange, float smallError, float smallErrorTimeout,
                           float largeError, float largeErrorTimeout, float slew, bool trapezoidal)
            : kP(kP),
              kI(kI),
              kD(kD),
              windupRange(windupRange),
              smallError(smallError),
              smallErrorTimeout(smallErrorTimeout),
              largeError(largeError),
              largeErrorTimeout(largeErrorTimeout),
              slew(slew),
              trapezoidal(trapezoidal) {}

        float kP;
        float kI;
        float kD;
        float windupRange;
        float smallError;
        float smallErrorTimeout;
        float largeError;
        float largeErrorTimeout;
        float slew;
        bool trapezoidal;
};

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

        void setGains(Gains gains);

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