#pragma once

#include <vector>
#include <cmath>
#include <initializer_list>
#include "pose.h"
#include "chassis/chassis.h"
#include "utils/utils.h"

template <typename T>
int sgn(T x) {
    return (T(0) < x) - (x < T(0));
}

template <typename T>
float avg(const std::vector<T>& values) {
    float sum = 0;
    for (const auto& value : values) {
        sum += value;
    }
    return sum / values.size();
}

template <typename T>
float avg(const std::initializer_list<T>& values) {
    float sum = 0;
    for (const auto& value : values) {
        sum += value;
    }
    return sum / values.size();
}

template <typename T>
float mod(T a, T b) {
    return a - b * floor(a / b);
}

/*** @brief only numbers!!!!!!!!!! ***/
template <typename T>
float avg(const std::vector<T>& values);

template <typename T>
float avg(const std::initializer_list<T>& values);

float rad2deg(float rad);

float deg2rad(float deg);

float ema(float newValue, float oldValue, float alpha);

Pose smoothPose(const Pose& newPose, const Pose& oldPose, float alpha);

float rot2inch(float circumference, float rot);

float deg2inch(float circumference, float deg);

float dist(float x1, float y1, float x2, float y2);

float sanitizeAngle(float angle, bool radians = false);

float angleError(float target, float position, bool radians = false, AngularDirection direction = AngularDirection::AUTO);

float angleErrorRaw(float target, float current);

float angleLerp(float a, float b, float t);

float slew(float target, float current, float maxChange);

float getCurvature(Pose pose, Pose other);

float curve(float input, float deadband, float curveGain, float minOutput = 0);

double project(Point point, Pose line);