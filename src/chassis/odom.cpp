#include "utils.h"
#include "robot-config.h"
#include "pros/rtos.hpp"
#include "chassis/miku/odom.h"
#include <vector>

// tracking thread
pros::Task* trackingTask = nullptr;

Pose odomPose(0, 0, 0);
Pose dPose(0, 0, 0);

float prev_s = 0;
float prev_v = 0;
float prev_imu = 0;
float prev_left = 0;
float prev_right = 0;

Pose getPose(bool radians) {
    if (radians) return odomPose;
    else return Pose(odomPose.x, odomPose.y, rad2deg(odomPose.theta));
}

void setPose(Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = Pose(pose.x, pose.y, deg2rad(pose.theta));
}

Pose getSpeed(bool radians) {
    if (radians) return dPose;
    else return Pose(dPose.x, dPose.y, rad2deg(dPose.theta));
}

// Pose estimatePose(float time, bool radians) {
//     // get current position and speed
//     Pose pose = getPose(true);
//     Pose speed = getSpeed(true);
//     // calculate the change in local position
//     Pose dPose = speed * time;

//     // calculate the future pose
//     float avgHeading = pose.theta + dPose.theta / 2;
//     Pose futurePose = pose;
//     futurePose.x += dPose.y * sin(avgHeading);
//     futurePose.y += dPose.y * cos(avgHeading);
//     futurePose.x += deltaLocalPose.x * -cos(avgHeading);
//     futurePose.y += deltaLocalPose.x * sin(avgHeading);
//     if (!radians) futurePose.theta = radToDeg(futurePose.theta);

//     return futurePose;
// }

void update() {
    float s_raw = deg2inch(hori.get_angle() / 100);
    float imuRaw = deg2rad(imu.get_rotation());
    float left_raw = avg(left_dt.get_position_all());
    float right_raw = avg(right_dt.get_position_all());
    float v_raw = avg({left_raw, right_raw}) - avg({prev_left, prev_right});
    float d_imu = imuRaw - prev_imu;

    prev_imu = imuRaw;
    prev_left = left_raw;
    prev_right = right_raw;

    float heading = odomPose.theta;
    
    heading += d_imu;
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    float deltaX = 0;
    float deltaY = 0;
    deltaY = rot2inch(M_PI * DT_WHEEL_DIAMETER, v_raw - prev_v);
    deltaX = rot2inch(M_PI * TRACKING_WHEEL_DIAMETER, s_raw - prev_s);
    

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + TRACKING_WHEEL_OFFSET);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading);
    }

    prev_s = s_raw;
    prev_v = v_raw;

    // save previous pose
    Pose prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    // calculate speed
    dPose.x = ema((odomPose.x - prevPose.x) / 0.01, dPose.x, 0.95);
    dPose.y = ema((odomPose.y - prevPose.y) / 0.01, dPose.y, 0.95);
    dPose.theta = ema((odomPose.theta - prevPose.theta) / 0.01, dPose.theta, 0.95);
    
}

void init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}