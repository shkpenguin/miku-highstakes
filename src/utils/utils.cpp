#include "utils.h"
#include "cmath"
#include <vector>
#include <initializer_list>
#include "utils/pose.h"
#include "utils/math.h"

float sanitizeAngle(float angle, bool radians) {
    if (radians) return std::fmod(std::fmod(angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    else return std::fmod(std::fmod(angle, 360) + 360, 360);
}

float angleError(float target, float position, bool radians, AngularDirection direction) {
    // bound angles from 0 to 2pi or 0 to 360
    target = sanitizeAngle(target, radians);
    position = sanitizeAngle(position, radians);
    const float max = radians ? 2 * M_PI : 360;
    const float rawError = target - position;
    switch (direction) {
        case AngularDirection::CW_CLOCKWISE: // turn clockwise
            return rawError < 0 ? rawError + max : rawError; // add max if sign does not match
        case AngularDirection::CCW_COUNTERCLOCKWISE: // turn counter-clockwise
            return rawError > 0 ? rawError - max : rawError; // subtract max if sign does not match
        default: // choose the shortest path
            return std::remainder(rawError, max);
    }
}

float slew(float target, float current, float maxChange) {
    float change = target - current;
    if (maxChange == 0) return target;
    if (change > maxChange) change = maxChange;
    else if (change < -maxChange) change = -maxChange;
    return current + change;
}

float getCurvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    float side = sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

float curve(float input, float deadband, float curveGain, float minOutput) {
    if (fabs(input) <= deadband) return 0;
    else {
        // g is the output of g(x) as defined in the Desmos graph
        const float g = fabs(input) - deadband;
        // g127 is the output of g(127) as defined in the Desmos graph
        const float g127 = 127 - deadband;
        // i is the output of i(x) as defined in the Desmos graph
        const float i = pow(curveGain, g - 127) * g * sgn(input);
        // i127 is the output of i(127) as defined in the Desmos graph
        const float i127 = pow(curveGain, g127 - 127) * g127;
        return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * sgn(input);
    }
}

std::string _2f(float num) {
    std::string str = std::to_string(num);
    size_t pos = str.find(".");
    if (pos != std::string::npos) {
        str = str.substr(0, pos + 3);
    }
    return str;
}