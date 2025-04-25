#pragma once

#include "api.h"
#include "pose.h"

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