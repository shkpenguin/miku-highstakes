#pragma once

#include "api.h"
#include "pose.h"

// velocity control

#define MAX_RPM 640
#define MAX_JERK 100.0

// odom settings

#define TRACK_WIDTH 14.0
#define DT_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 2.75
#define VERTICAL_OFFSET 0.0
#define HORIZONTAL_OFFSET -2.5
#define HORIZONTAL_DRIFT 10.0

// mcl

#define HORIZONTAL_DIST_OFFSET 12
#define VERTICAL_DIST_OFFSET 2

// arcade control settings
#define DEADBAND 5
#define MIN_SPEED 10
#define EXPO_CURVE_GAIN 1.019

// ramsete settings
#define K_V 1.0
#define K_W 2.0

enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

enum Color {
    RED = 1,
    BLUE = -1,
    NONE = 0
};

/**
* @brief i am not installing fmt just for this
**/
std::string _2f(float num);

std::string _1f(float num);