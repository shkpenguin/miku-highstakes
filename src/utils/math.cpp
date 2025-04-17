#include "utils/math.h"

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

float rad2deg(float rad) {
    return rad * 180 / M_PI;
}

float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float ema(float newValue, float oldValue, float alpha) {
    return (alpha * newValue) + ((1 - alpha) * oldValue);
}

float rot2inch(float circumference, float rotations) {
    return (circumference * rotations);
}

float deg2inch(float circumference, float deg) {
    return (circumference * deg) / 360;
}