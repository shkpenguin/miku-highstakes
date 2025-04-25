#include "utils/utils.h"
#include "robot-config.h"
#include "pros/rtos.hpp"
#include "chassis/odom.h"
#include "mcl.h"
#include <vector>

Pose prevPose(0, 0, 0.0);
Pose odomPose(0, 0, 0.0);
Pose dPose(0, 0, 0);

float prev_hori = 0;
float prev_vertical = 0;
float prev_imu = 0;

pros::Task* trackingTask = nullptr;

Pose getPose(bool radians, bool standardPos) {
    Pose pose = odomPose;
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = rad2deg(pose.theta);
    return pose;
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
    float hori_raw = deg2inch(TRACKING_WHEEL_DIAMETER * M_PI, hori.get_position() / 100);
    float vert_raw = deg2inch(TRACKING_WHEEL_DIAMETER * M_PI, vertical.get_position() / 100);
    float imuRaw = deg2rad(imu.get_rotation());
    float d_heading = imuRaw - prev_imu;

    prev_imu = imuRaw;

    float heading = odomPose.theta;

    heading += d_heading;
    float avgHeading = odomPose.theta + d_heading / 2;

    float deltaX = 0;
    float deltaY = 0;
    deltaY = vert_raw - prev_vertical;
    deltaX = hori_raw - prev_hori;

    // Safeguard against division by zero
    float localX = 0;
    float localY = 0;
    if (std::abs(d_heading) == 0) { 
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(d_heading / 2) * (deltaX / d_heading + HORIZONTAL_OFFSET);
        localY = 2 * sin(d_heading / 2) * (deltaY / d_heading + VERTICAL_OFFSET);
    }

    prev_hori = hori_raw;
    prev_vertical = vert_raw;

    // Save previous pose
    prevPose = odomPose;

    dPose.x = localY * std::sin(avgHeading);
    dPose.y = localY * std::cos(avgHeading);
    dPose.x += localX * -std::cos(avgHeading);
    dPose.y += localX * std::sin(avgHeading);

    // Calculate global x and y
    odomPose.x += dPose.x;
    odomPose.y += dPose.y;
    odomPose.theta = heading;

}

void initOdom(Pose start) {
    setPose(start);
    trackingTask = new pros::Task{[=] {
	while (true) {
		update();
        motionUpdate(Point(getSpeed().x, getSpeed().y));

        std::vector<float> sensors = {
            static_cast<float>(leftDist.get() / 25.4),
            static_cast<float>(rightDist.get() / 25.4)
        };

        bool valid = sensorUpdate(sensors);

        if (valid) {
            Pose estPose(getEstimate().x, getEstimate().y, getPose(true).theta);
            setPose(estPose, true);
            resampleParticles();
            injectAroundEstimate(0.5, 5.0);  // Only when we have good data
        }

		pros::delay(10);
	}
}};
}