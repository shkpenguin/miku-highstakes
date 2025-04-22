#pragma once

#include "api.h"
#include "pose.h"

enum LBState {
    DOWN = 0,
    READY = 4200,
    RAISED = -1,
    DESCORE = 16500,
    HANG = 8000
};

enum RingState {
    COLOR = 0,
    DIST = 1,
    OUT = 2
};

enum Color {
    RED = 1,
    BLUE = -1,
    NONE = 0
};

enum class AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

float sanitizeAngle(float angle, bool radians = false);

float angleError(float target, float position, bool radians = false, AngularDirection direction = AngularDirection::AUTO);

float slew(float target, float current, float maxChange);

float getCurvature(Pose pose, Pose other);

float curve(float input, float deadband, float curveGain, float minOutput = 0);

/**
* @brief i am not installing fmt just for this
**/
std::string _2f(float num);