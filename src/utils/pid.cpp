#include "pid.h"
#include "utils.h"
#include "math.h"

PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset, bool trapezoidal)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset),
      trapezoidal(trapezoidal) {}

float PID::update(const float error) {

    // calculate derivative
    const float derivative = error - prevError;

    // calculate integral
    if (trapezoidal) {
        if (sgn(derivative) != sgn(error)) {
            integral += (error + prevError) / 2;
        } else {
            integral += error;
        }
    } else {
        integral += error;
    }
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;
    
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}