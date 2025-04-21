#include "utils/math.h"
#include "utils/utils.h"

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