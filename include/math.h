#pragma once

#include "api.h"

int sgn(int x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}
int sgn(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}
int sgn(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

