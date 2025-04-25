#include "chassis/chassis.h"
#include "utils/math.h"
#include "utils/utils.h"
#include "robot-config.h"
#include "chassis/tbh.h"
#include "utils/timer.h"
#include "chassis/odom.h"

//k1 - aggressive
//k2 - smooth
//k3 - turning aggressiveness
void Chassis::movePose(float x, float y, float theta, int timeout, MovePoseParams params, bool async) {
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { movePose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    Timer timer(timeout);
    tbh.reset();
    theta = deg2rad(mod(90 - theta, 360.0f));
    tbh.setStatus(true);
    bool first = true;
    std::uint32_t now = pros::millis();
    
    while (dist(x, y, getPose().x, getPose().y) > params.cutoff && !timer.isDone()) {
        float rh = getPose(true).theta;
        if (!params.forwards) {
            rh = mod(rh + M_PI, 2 * M_PI);
        }

        float rho = dist(x, y, getPose().x, getPose().y);
        float gamma = round(mod(std::atan2(y - getPose().y, x - getPose().x) - rh, static_cast<float>(2 * M_PI)));
        float delta = round(mod(std::atan2(y - getPose().y, x - getPose().x) - theta, static_cast<float>(2 * M_PI)));

        float v = params.k1 * rho * std::cos(gamma);
        float w;
        if (std::abs(gamma) <= 0.01) {
            w = params.k2 * gamma + params.k1 * std::cos(gamma) * (gamma + params.k3 * delta);
        } else {
            w = params.k2 * gamma + params.k1 * std::sin(gamma) * std::cos(gamma) / gamma * (gamma + params.k3 * delta);
        }

        if (std::abs(v) > params.maxSpeed) {
            v = sgn(v) * params.maxSpeed;
            w *= params.maxSpeed / std::abs(v);
        } else if (std::abs(v) < params.minSpeed) {
            v = sgn(v) * params.minSpeed;
        }

        float r_vel = v + TRACK_WIDTH * w / 2;
        float l_vel = v - TRACK_WIDTH * w / 2;

        float m = std::max(std::abs(r_vel), std::abs(l_vel));
        if (m > 100) {
            r_vel *= 100 / m;
            l_vel *= 100 / m;
        }

        if (!params.forwards) {
            tbh.setRightTarget(-1 * l_vel);
            tbh.setRightTarget(-1 * r_vel);
            if (first) {
                tbh.setRightVolts(voltageLookup(-1 * l_vel));
                tbh.setLeftVolts(voltageLookup(-1 * r_vel));
                first = false;
            }
        } else {
            tbh.setRightTarget(r_vel);
            tbh.setLeftTarget(l_vel);
            if (first) {
                tbh.setRightVolts(voltageLookup(r_vel));
                tbh.setLeftVolts(voltageLookup(l_vel));
            }
        }
        
        pros::delay(10);
    }

    tbh.setStatus(false);

    movePoint(x, y, timer.getTimeLeft(), MovePointParams{params.forwards, params.maxSpeed, params.minSpeed, }, false);
}