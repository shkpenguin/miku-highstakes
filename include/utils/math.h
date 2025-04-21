#pragma once

#include <vector>
#include <cmath>
#include <initializer_list>

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

/*** @brief only numbers!!!!!!!!!! ***/
template <typename T>
float avg(const std::vector<T>& values);

template <typename T>
float avg(const std::initializer_list<T>& values);

float rad2deg(float rad);

float deg2rad(float deg);

float ema(float newValue, float oldValue, float alpha);

float rot2inch(float circumference, float rot);

float deg2inch(float circumference, float deg);

