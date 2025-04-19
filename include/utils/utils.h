#pragma once

#include "api.h"

enum LBState {
    DOWN = 0,
    READY = 1,
    RAISED = 2,
    DESCORE = 3
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

enum FieldState {
    INITIALIZE = 0,
    COMPETITION = 1,
    AUTONOMOUS = 2,
    DISABLED = 3,
    OPCONTROL = 4
};

enum AngularDirection {
    CW_CLOCKWISE, /** turn clockwise */
    CCW_COUNTERCLOCKWISE, /** turn counter-clockwise */
    AUTO /** turn in the direction with the shortest distance to target */
};

float sanitizeAngle(float angle, bool radians = false);

float angleError(float target, float position, bool radians = false, AngularDirection direction = AngularDirection::AUTO);

float slew(float target, float current, float maxChange);

float getCurvature(Pose pose, Pose other);